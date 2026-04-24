package com.adambots.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.lib.utils.Dash;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionCamera;
import com.adambots.lib.vision.VisionResult;
import com.adambots.lib.vision.VisionTarget;
import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;
import com.adambots.lib.vision.config.VisionConfigBuilder;
import com.adambots.lib.vision.config.VisionSystemConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import static com.adambots.logging.LogUtil.DIAGNOSTIC;
import static com.adambots.logging.LogUtil.DEBUG;
import static com.adambots.logging.LogUtil.ESSENTIAL;
import static com.adambots.logging.LogUtil.log;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Vision subsystem using AdambotsLib's PhotonVision for AprilTag-based distance and angle estimation.
 * Supports three vision modes (selectable at runtime via Shuffleboard):
 * <ul>
 *   <li><b>Mode 0 — Camera-only:</b> direct tag geometry via shooter_cam (requires shooter camera)</li>
 *   <li><b>Mode 1 — Pose-only:</b> pose-based via lib's PhotonVision using swerve odometry (requires back cameras)</li>
 *   <li><b>Mode 2 — Hybrid:</b> camera is primary, automatically falls back to pose when camera can't see the hub</li>
 * </ul>
 *
 * <p>Camera availability is controlled by per-camera flags in RobotMap:
 * <ul>
 *   <li>{@code BACK_CAMERAS_ENABLED} — back-left and back-right ArduCams for odometry/pose estimation</li>
 *   <li>{@code SHOOTER_CAMERA_ENABLED} — shooter LifeCam for hub camera-only tracking</li>
 * </ul>
 *
 * <p>Hub tracking runs every cycle in periodic() — distance and angle are computed
 * to the geometric center of the alliance hub tags. Other target groups (trench, tower, etc.)
 * can be checked for visibility via {@link #areTagsVisible(int[])} and {@link #tagsVisibleTrigger(int[])}.
 *
 * <p>A Shuffleboard numeric widget ("Vision Mode" on the Vision tab) switches the active mode.
 * Default is set by {@link VisionConstants#kVisionMode}.
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonVision photonVision;
    private final Supplier<Pose2d> poseSupplier;

    // Which cameras are available
    private final boolean hasBackCameras;
    private final boolean hasShooterCamera;

    // Turret angle supplier — needed to convert pose angle (robot-relative) to turret-relative
    private DoubleSupplier turretAngleSupplier = () -> TurretConstants.kTurretForwardDegrees;

    // Precomputed hub centers (geometric center of all hub tags per alliance)
    private final Translation2d blueHubCenter;
    private final Translation2d redHubCenter;

    // Hub tracking — Approach A outputs (camera-only via shooter_cam)
    private double hubCamDistanceMeters = 0;
    private double hubCamAngleDegrees = 0;
    private boolean hubCamHasTarget = false;

    // Hub tracking — Approach B outputs (pose-based via swerve odometry)
    private double hubPoseDistanceMeters = 0;
    private double hubPoseAngleDegrees = 0;
    private boolean hubPoseHasTarget = false;

    // Hub tracking — Mode 3 outputs (blended: weighted avg of camera + pose)
    private double hubBlendedDistanceMeters = 0;
    private double hubBlendedAngleDegrees = 0;

    // Hub tracking — shared
    private int hubVisibleTagCount = 0;
    private boolean prevHubCamHasTarget = false;
    private boolean prevHubPoseHasTarget = false;

    // Hub camera holdoff: charge/decay counter makes isHubCamVisible() "sticky"
    // so consumers don't see per-frame flicker from intermittent detections.
    private int hubCamHoldoffCounter = 0;
    private boolean hubCamSticky = false;
    private double lastHubCamAngle = 0;

    // Raw (pre-filter) values for diagnostic logging
    private double rawCamDist, rawCamAngle;
    private double rawPoseDist, rawPoseAngle;

    // Exponential weighted average (EWA) state — NaN = not yet initialized (cold-start seed)
    private double ewaCamDist = Double.NaN;
    private double ewaCamAngle = Double.NaN;
    private double ewaPoseDist = Double.NaN;

    // Runtime vision mode (editable from Shuffleboard): 0=Camera-only, 1=Pose-only, 2=Hybrid
    private int visionMode = VisionConstants.kVisionMode;

    // Tunable ambiguity threshold (default from constants, overridable via TuningManager)
    private double runtimeAmbiguityThreshold = VisionConstants.kAmbiguityThreshold;

    // Diagnostic counters for camera-only target processing
    private int diagTagsSeen = 0;
    private int diagTagsRejectedAmbiguity = 0;
    private int diagTagsRejectedNotHub = 0;
    private double diagLastAmbiguity = -1;

    // Camera health tracking — detects pipeline offline (USB disconnect, driver crash)
    private double lastCameraResultTime = 0;
    private boolean cameraOnline = true;

    // Throttled logging (1 Hz)
    private double lastLogTimestamp = 0;

    /**
     * Creates a VisionSubsystem with the specified cameras enabled.
     *
     * @param poseSupplier supplies the current robot pose from swerve odometry
     * @param field Field2d for dashboard visualization
     * @param backCamerasEnabled whether back-left and back-right odometry cameras are present
     * @param shooterCameraEnabled whether the shooter alignment camera is present
     */
    public VisionSubsystem(Supplier<Pose2d> poseSupplier, Field2d field,
                           boolean backCamerasEnabled, boolean shooterCameraEnabled,
                           boolean frontCameraEnabled) {
        this.poseSupplier = poseSupplier;
        this.hasBackCameras = backCamerasEnabled;
        this.hasShooterCamera = shooterCameraEnabled;

        // Build vision config with only the enabled cameras
        VisionConfigBuilder builder = VisionConfigBuilder.create();

        if (hasBackCameras) {
            // Back-left ArduCam (on back-left swerve module, facing backward)
            builder.addCamera(VisionConstants.kBackLeftCameraName)
                .position(Meters.of(VisionConstants.kBackLeftCameraX),
                          Meters.of(VisionConstants.kBackLeftCameraY),
                          Meters.of(VisionConstants.kBackLeftCameraZ))
                .rotation(Degrees.of(VisionConstants.kBackLeftCameraRoll),
                          Degrees.of(VisionConstants.kBackLeftCameraPitch),
                          Degrees.of(VisionConstants.kBackLeftCameraYaw))
                .purpose(CameraPurpose.ODOMETRY)
                .singleTagStdDevs(Meters.of(VisionConstants.kSingleTagStdDevs[0]),
                                  Meters.of(VisionConstants.kSingleTagStdDevs[1]),
                                  Radians.of(VisionConstants.kSingleTagStdDevs[2]))
                .multiTagStdDevs(Meters.of(VisionConstants.kMultiTagStdDevs[0]),
                                 Meters.of(VisionConstants.kMultiTagStdDevs[1]),
                                 Radians.of(VisionConstants.kMultiTagStdDevs[2]))
                .maxTagDistance(Meters.of(VisionConstants.kOdomMaxTagDistance))
                .minTagArea(VisionConstants.kOdomMinTagAreaPercent)
                .done();

            // Back-right ArduCam (on back-right swerve module, facing backward)
            builder.addCamera(VisionConstants.kBackRightCameraName)
                .position(Meters.of(VisionConstants.kBackRightCameraX),
                          Meters.of(VisionConstants.kBackRightCameraY),
                          Meters.of(VisionConstants.kBackRightCameraZ))
                .rotation(Degrees.of(VisionConstants.kBackRightCameraRoll),
                          Degrees.of(VisionConstants.kBackRightCameraPitch),
                          Degrees.of(VisionConstants.kBackRightCameraYaw))
                .purpose(CameraPurpose.ODOMETRY)
                .singleTagStdDevs(Meters.of(VisionConstants.kSingleTagStdDevs[0]),
                                  Meters.of(VisionConstants.kSingleTagStdDevs[1]),
                                  Radians.of(VisionConstants.kSingleTagStdDevs[2]))
                .multiTagStdDevs(Meters.of(VisionConstants.kMultiTagStdDevs[0]),
                                 Meters.of(VisionConstants.kMultiTagStdDevs[1]),
                                 Radians.of(VisionConstants.kMultiTagStdDevs[2]))
                .maxTagDistance(Meters.of(VisionConstants.kOdomMaxTagDistance))
                .minTagArea(VisionConstants.kOdomMinTagAreaPercent)
                .done();
        }

        if (hasShooterCamera) {
            // Shooter LifeCam (on top of shooter at back, facing forward)
            builder.addCamera(VisionConstants.kShooterCameraName)
                .position(Meters.of(VisionConstants.kShooterCameraX),
                          Meters.of(VisionConstants.kShooterCameraY),
                          Meters.of(VisionConstants.kShooterCameraZ))
                .rotation(Degrees.of(VisionConstants.kShooterCameraRoll),
                          Degrees.of(VisionConstants.kShooterCameraPitch),
                          Degrees.of(VisionConstants.kShooterCameraYaw))
                .purpose(CameraPurpose.ALIGNMENT)
                .singleTagStdDevs(Meters.of(VisionConstants.kSingleTagStdDevs[0]),
                                  Meters.of(VisionConstants.kSingleTagStdDevs[1]),
                                  Radians.of(VisionConstants.kSingleTagStdDevs[2]))
                .multiTagStdDevs(Meters.of(VisionConstants.kMultiTagStdDevs[0]),
                                 Meters.of(VisionConstants.kMultiTagStdDevs[1]),
                                 Radians.of(VisionConstants.kMultiTagStdDevs[2]))
                .maxTagDistance(Meters.of(VisionConstants.kAlignMaxTagDistance))
                .done();
        }

        if (frontCameraEnabled) {
            // Front LifeCam (forward-facing, pitched 27° up for hub tag visibility)
            builder.addCamera(VisionConstants.kFrontCameraName)
                .position(Meters.of(VisionConstants.kFrontCameraX),
                          Meters.of(VisionConstants.kFrontCameraY),
                          Meters.of(VisionConstants.kFrontCameraZ))
                .rotation(Degrees.of(VisionConstants.kFrontCameraRoll),
                          Degrees.of(VisionConstants.kFrontCameraPitch),
                          Degrees.of(VisionConstants.kFrontCameraYaw))
                .purpose(CameraPurpose.ODOMETRY)
                .singleTagStdDevs(Meters.of(VisionConstants.kSingleTagStdDevs[0]),
                                  Meters.of(VisionConstants.kSingleTagStdDevs[1]),
                                  Radians.of(VisionConstants.kSingleTagStdDevs[2]))
                .multiTagStdDevs(Meters.of(VisionConstants.kMultiTagStdDevs[0]),
                                 Meters.of(VisionConstants.kMultiTagStdDevs[1]),
                                 Radians.of(VisionConstants.kMultiTagStdDevs[2]))
                .maxTagDistance(Meters.of(VisionConstants.kOdomMaxTagDistance))
                .minTagArea(VisionConstants.kOdomMinTagAreaPercent)
                .done();
        }

        VisionSystemConfig config = builder
            .ambiguityThreshold(VisionConstants.kAmbiguityThreshold)
            .maxPoseJump(Meters.of(2.0))
            .maxHeadingJump(Degrees.of(45.0))
            .build();

        // Create PhotonVision with swerve odometry pose supplier
        photonVision = new PhotonVision(config, poseSupplier, field);

        // Precompute hub centers (geometric center of all hub tags per alliance)
        blueHubCenter = photonVision.getTagGroupCenter(VisionConstants.kBlueHubTags);
        redHubCenter = photonVision.getTagGroupCenter(VisionConstants.kRedHubTags);

        if (blueHubCenter.getNorm() == 0 || redHubCenter.getNorm() == 0) {
            System.err.println("[Vision] WARNING: Hub center is at field origin — check tag IDs and field layout JSON");
        }

        // If shooter camera is missing, clamp to pose-only mode
        if (!hasShooterCamera) {
            visionMode = 1;
        }

        System.out.printf("[Vision] Init: redHub=(%.2f,%.2f) blueHub=(%.2f,%.2f) shooterCam=%s backCams=%s ambigThresh=%.2f%n",
            redHubCenter.getX(), redHubCenter.getY(),
            blueHubCenter.getX(), blueHubCenter.getY(),
            hasShooterCamera ? "present" : "absent",
            hasBackCameras ? "present" : "absent",
            VisionConstants.kAmbiguityThreshold);

        if (Constants.VISION_TAB) setupDash();
        // Hub distance on default Shuffleboard — useful for pit crew / coach quick glance.
        // Gated behind TUNING_ENABLED to avoid an always-on NT supplier at comp
        // (the same value is logged to AdvantageKit as Vision/OutputDist at ESSENTIAL).
        if (Constants.TUNING_ENABLED) {
            Dash.add("Dist (m)", this::getHubDistance);
        }
    }

    private void setupDash() {
        Dash.useTab("Vision");
        int col, row;

        // Row 0: Hub unified outputs (active approach)
        col = 0; row = 0;
        Dash.add("Hub Distance (m)", this::getHubDistance, col++, row);
        Dash.add("Pose Dist (m)", this::getPoseBasedDistance, col++, row);
        Dash.add("Hub Angle (deg)", this::getHubAngle, col++, row);
        Dash.add("Hub Visible", this::isHubVisible, col++, row);
        Dash.add("Hub Tags", this::getHubVisibleTagCount, col++, row);
        Dash.add("Alliance", this::getAllianceColor, col++, row);

        // Row 1: Hub camera-only (Approach A) + diagnostics
        if (hasShooterCamera) {
            col = 0; row = 1;
            Dash.add("Hub Cam Distance", this::getHubCamDistance, col++, row);
            Dash.add("Hub Cam Angle", this::getHubCamAngle, col++, row);
            Dash.add("Hub Cam Visible", this::isHubCamVisible, col++, row);
            Dash.add("Cam Tags Seen", () -> diagTagsSeen, col++, row);
            Dash.add("Rejected (Ambig)", () -> diagTagsRejectedAmbiguity, col++, row);
            Dash.add("Rejected (ID)", () -> diagTagsRejectedNotHub, col++, row);
            Dash.add("Last Ambiguity", () -> diagLastAmbiguity, col++, row);
        }

        // Row 2: Hub pose-based (Approach B) — only if back cameras are present
        if (hasBackCameras) {
            col = 0; row = 2;
            Dash.add("Hub Pose Distance", this::getHubPoseDistance, col++, row);
            Dash.add("Hub Pose Angle", this::getHubPoseAngle, col++, row);
            Dash.add("Hub Pose Visible", this::isHubPoseVisible, col++, row);
        }

        // Row 3: Diagnostic telemetry for debugging alliance/hub issues in pit
        col = 0; row = 3;
        Dash.add("Alliance (detected)", this::getAllianceColor, col++, row);
        Dash.add("Hub Center X", () -> getHubCenter().getX(), col++, row);
        Dash.add("Hub Center Y", () -> getHubCenter().getY(), col++, row);
        Dash.add("Pose X", () -> poseSupplier.get().getX(), col++, row);
        Dash.add("Pose Y", () -> poseSupplier.get().getY(), col++, row);

        Dash.useDefaultTab();
    }

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setVisionMode(int mode) {
        mode = Math.max(0, Math.min(3, mode)); // clamp to valid range
        visionMode = hasShooterCamera ? mode : 1;
    }

    public void setAmbiguityThreshold(double threshold) {
        runtimeAmbiguityThreshold = threshold;
    }

    /** Set turret angle supplier so blended mode can convert pose angle to turret-relative. */
    public void setTurretAngleSupplier(DoubleSupplier supplier) {
        turretAngleSupplier = supplier;
    }

    /**
     * Enable/disable shooting mode tag filter. When enabled, pose estimation
     * only uses hub tags — prevents non-hub tags from corrupting the pose during shots.
     */
    public void setShootingMode(boolean shooting) {
        if (shooting) {
            int[] hubTags = Utils.isOnRedAlliance()
                ? VisionConstants.kRedHubTags : VisionConstants.kBlueHubTags;
            photonVision.setTagFilter(hubTags);
        } else {
            photonVision.setTagFilter(null);
        }
    }

    // Raw value getters for DashboardSetup to display behind TUNING_ENABLED
    public double getRawCamDist() { return rawCamDist; }
    public double getRawCamAngle() { return rawCamAngle; }
    public double getRawPoseDist() { return rawPoseDist; }
    public double getRawPoseAngle() { return rawPoseAngle; }

    @Override
    public void periodic() {
        if (Constants.TUNING_ENABLED) {
            SmartDashboard.putNumber("Vision/Mode", visionMode);
        }

        // Pose estimation is handled by SwerveSubsystem.periodic() via swerve.setupVision(vision).
        // Do NOT call updatePoseEstimation() here — PhotonPoseEstimator's timestamp cache
        // would consume the result, preventing swerve from receiving vision measurements.

        // Resolve alliance hub tags and center
        boolean isRed = Utils.isOnRedAlliance();
        Translation2d hubCenter = isRed ? redHubCenter : blueHubCenter;
        int[] hubTagIds = isRed ? VisionConstants.kRedHubTags : VisionConstants.kBlueHubTags;

        // Hub tag count
        hubVisibleTagCount = photonVision.getVisibleTagCount(hubTagIds, runtimeAmbiguityThreshold);

        // ==================== Hub Approach A: Camera-Only ====================
        if (hasShooterCamera) {
            updateHubCameraOnly(hubCenter, hubTagIds);
        }

        // Charge/decay holdoff: makes isHubCamVisible() sticky across frame gaps
        if (hubCamHasTarget) {
            hubCamHoldoffCounter = Math.min(
                hubCamHoldoffCounter + VisionConstants.kCamChargeRate,
                VisionConstants.kCamCounterMax);
            lastHubCamAngle = hubCamAngleDegrees;
        } else {
            hubCamHoldoffCounter = Math.max(hubCamHoldoffCounter - 1, 0);
        }
        hubCamSticky = hubCamHoldoffCounter > 0;
        if (Constants.TUNING_ENABLED) {
            SmartDashboard.putNumber("Vision/HoldoffCounter", hubCamHoldoffCounter);
            SmartDashboard.putBoolean("Vision/HubSticky", hubCamSticky);
            SmartDashboard.putBoolean("Vision/HubFresh", hubCamHasTarget);
        }

        // ==================== Hub Approach B: Pose-Based ====================
        // Only compute when visionMode uses pose data (1=Pose-only, 2=Hybrid)
        if (hasBackCameras && visionMode != 0) {
            Pose2d currentPose = poseSupplier.get();
            // Guard: skip if swerve pose is still at origin (no vision updates processed yet).
            // No camera dependency — Approach B is purely pose-based via swerve odometry.
            if (currentPose.getTranslation().getNorm() > 0) {
                double rawDist = photonVision.getDistanceToPoint(hubCenter);
                double rawAngle = photonVision.getYawToPoint(hubCenter).getDegrees();
                rawPoseDist = rawDist;
                rawPoseAngle = rawAngle;
                // Distance: EWA (seed on first measurement, avoids cold-start lag)
                ewaPoseDist = Double.isNaN(ewaPoseDist) ? rawDist : ewaPoseDist + VisionConstants.kVisionAlpha * (rawDist - ewaPoseDist);
                hubPoseDistanceMeters = ewaPoseDist;
                // Angle: use raw value — the swerve pose estimator already smooths the pose,
                // and linear filters break at the ±180° wraparound boundary causing wild oscillation.
                hubPoseAngleDegrees = rawAngle;
                hubPoseHasTarget = hubPoseDistanceMeters <= VisionConstants.kMaxDistanceMeters;
            } else {
                hubPoseHasTarget = false;
            }

            // Single reset point: true→false transition (covers both pose-at-origin and out-of-range)
            if (!hubPoseHasTarget && prevHubPoseHasTarget) {
                ewaPoseDist = Double.NaN;
            }
            prevHubPoseHasTarget = hubPoseHasTarget;
        }

        // Mode 3: compute weighted blend of camera and pose values.
        // Camera angle is turret-relative (offset from boresight).
        // Pose angle is robot-relative (bearing from robot heading to hub).
        // Convert pose angle to turret-relative before blending:
        //   turretRelativePoseAngle = poseAngle - (turretAngle - turretForwardDeg)
        double poseAngleTurretRelative = poseAngleToTurretRelative(hubPoseAngleDegrees);
        if (hubCamHasTarget && hubPoseHasTarget) {
            // Disagreement guard: if camera and pose angles differ by more than threshold,
            // the sources are inconsistent (possible miscalibration or ambiguous tag).
            // Fall back to camera-only which is the more direct measurement.
            if (Math.abs(hubCamAngleDegrees - poseAngleTurretRelative) > VisionConstants.kBlendDisagreementThreshold) {
                hubBlendedDistanceMeters = hubCamDistanceMeters;
                hubBlendedAngleDegrees = hubCamAngleDegrees;
            } else {
                double w = VisionConstants.kVisionBlendWeight;
                hubBlendedDistanceMeters = w * hubCamDistanceMeters + (1.0 - w) * hubPoseDistanceMeters;
                hubBlendedAngleDegrees = w * hubCamAngleDegrees + (1.0 - w) * poseAngleTurretRelative;
            }
        } else if (hubCamHasTarget) {
            hubBlendedDistanceMeters = hubCamDistanceMeters;
            hubBlendedAngleDegrees = hubCamAngleDegrees;
        } else if (hubPoseHasTarget) {
            hubBlendedDistanceMeters = hubPoseDistanceMeters;
            hubBlendedAngleDegrees = poseAngleTurretRelative;
        }

        // ==================== Structured logging (AdvantageScope) ====================
        // 3-tier split: ESSENTIAL = things we need to diagnose "did vision work?" at comp,
        // DIAGNOSTIC = pipeline internals for tuning, DEBUG = EWMA intermediates + conversions.
        //
        // ESSENTIAL (always on at comp — covers MICMP1's Vision blackout gap):
        log(ESSENTIAL, "Vision/CamOnline", cameraOnline);
        log(ESSENTIAL, "Vision/CamHasTarget", hubCamHasTarget);
        log(ESSENTIAL, "Vision/HubTagCount", hubVisibleTagCount);
        log(ESSENTIAL, "Vision/Mode", visionMode);
        log(ESSENTIAL, "Vision/OutputAngle", getHubAngle());
        log(ESSENTIAL, "Vision/OutputDist", getHubDistance());
        log(ESSENTIAL, "Vision/OutputVisible", isHubVisible());
        log(ESSENTIAL, "Vision/OutputFresh", isTrackingDataFresh());

        // DIAGNOSTIC (bench / practice): pipeline internals, raw-vs-filtered pairs,
        // rejection counters, pose-based approach outputs.
        if (DIAGNOSTIC.enabled()) {
            Pose2d currentPose = poseSupplier.get();
            log(DIAGNOSTIC, "Vision/CamRawAngle", rawCamAngle);
            log(DIAGNOSTIC, "Vision/CamEWMAAngle", hubCamAngleDegrees);
            log(DIAGNOSTIC, "Vision/CamRawDist", rawCamDist);
            log(DIAGNOSTIC, "Vision/CamEWMADist", hubCamDistanceMeters);
            log(DIAGNOSTIC, "Vision/CamSticky", hubCamSticky);
            log(DIAGNOSTIC, "Vision/CamTagsSeen", diagTagsSeen);
            log(DIAGNOSTIC, "Vision/CamTagsRejectedAmbig", diagTagsRejectedAmbiguity);
            log(DIAGNOSTIC, "Vision/CamLastAmbiguity", diagLastAmbiguity);
            log(DIAGNOSTIC, "Vision/PoseX", currentPose.getX());
            log(DIAGNOSTIC, "Vision/PoseY", currentPose.getY());
            log(DIAGNOSTIC, "Vision/PoseHeading", currentPose.getRotation().getDegrees());
            log(DIAGNOSTIC, "Vision/PoseRawAngle", rawPoseAngle);
            log(DIAGNOSTIC, "Vision/PoseRawDist", rawPoseDist);
            log(DIAGNOSTIC, "Vision/PoseFilteredDist", hubPoseDistanceMeters);
            log(DIAGNOSTIC, "Vision/PoseAngle", hubPoseAngleDegrees);
            log(DIAGNOSTIC, "Vision/PoseHasTarget", hubPoseHasTarget);
            log(DIAGNOSTIC, "Vision/PoseBasedDist", getPoseBasedDistance());
            log(DIAGNOSTIC, "Vision/BlendedAngle", hubBlendedAngleDegrees);
            log(DIAGNOSTIC, "Vision/BlendedDist", hubBlendedDistanceMeters);
        }

        // DEBUG (bench only): turret-relative conversions, secondary references
        log(DEBUG, "Vision/TurretAngle", turretAngleSupplier.getAsDouble());
        log(DEBUG, "Vision/PoseAngleTurretRelative", poseAngleTurretRelative);

        // ==================== Console log (1 Hz, DIAGNOSTIC only) ====================
        // Human-readable summary for RioLog / driver station quick check. Off at comp
        // (LOG_LEVEL=ESSENTIAL) to avoid stdout spam; on at bench.
        if (DIAGNOSTIC.enabled()) {
            double now2 = Timer.getFPGATimestamp();
            if (now2 - lastLogTimestamp >= 1.0) {
                lastLogTimestamp = now2;
                String modeName = (visionMode >= 0 && visionMode <= 3) ? MODE_NAMES[visionMode] : "?";
                Translation2d hc = isRed ? redHubCenter : blueHubCenter;
                Pose2d logPose = poseSupplier.get();
                System.out.printf(
                    "[Vision] mode=%s cam=%s(seen=%d ambig=%d notHub=%d lastA=%.2f) "
                    + "pose=%s dist=%.2f/%.2f ang=%.1f/%.1f "
                    + "poseRel=%.1f turretAng=%.1f "
                    + "blend=%.1f/%.2f out=%.1f/%.2f "
                    + "tags=%d pose=(%.1f,%.1f,%.0f°) hub=(%.1f,%.1f)%n",
                    modeName,
                    hubCamHasTarget ? "T" : "F", diagTagsSeen,
                    diagTagsRejectedAmbiguity, diagTagsRejectedNotHub, diagLastAmbiguity,
                    hubPoseHasTarget ? "T" : "F",
                    hubCamDistanceMeters, hubPoseDistanceMeters,
                    hubCamAngleDegrees, hubPoseAngleDegrees,
                    poseAngleTurretRelative, turretAngleSupplier.getAsDouble(),
                    hubBlendedAngleDegrees, hubBlendedDistanceMeters,
                    getHubAngle(), getHubDistance(),
                    hubVisibleTagCount,
                    logPose.getX(), logPose.getY(),
                    logPose.getRotation().getDegrees(),
                    hc.getX(), hc.getY());
            }
        }
    }

    /**
     * Hub Approach A: Compute camera field pose from individual hub tag transforms,
     * then derive distance and angle to the hub center.
     * Uses shooter_cam only — the lib doesn't have this camera-only pattern.
     */
    private void updateHubCameraOnly(Translation2d hubCenter, int[] hubTagIds) {
        // Use getVisionCamera() (concrete type) so we can call getEstimatedGlobalPose()
        // to populate the results cache. ALIGNMENT cameras are skipped by the lib's
        // updatePoseEstimation(), so their resultsList is never populated automatically.
        VisionCamera cam = photonVision.getVisionCamera(VisionConstants.kShooterCameraName);
        if (cam == null) {
            hubCamHasTarget = false;
            if (prevHubCamHasTarget) {
                ewaCamDist = Double.NaN;
                ewaCamAngle = Double.NaN;
            }
            prevHubCamHasTarget = false;
            return;
        }
        // Trigger results fetch — populates internal resultsList from NetworkTables.
        // The returned pose is ignored (static transform is wrong for turret-mounted camera).
        cam.getEstimatedGlobalPose();
        Optional<? extends VisionResult> resultOpt = cam.getLatestResult();

        // Camera health: track last time we got any result (even with no targets)
        double now = Timer.getFPGATimestamp();
        if (resultOpt.isPresent()) {
            lastCameraResultTime = now;
            cameraOnline = true;
        } else if (lastCameraResultTime > 0
                && now - lastCameraResultTime > VisionConstants.kCameraOfflineThresholdSeconds) {
            cameraOnline = false;
        }

        if (resultOpt.isEmpty() || !resultOpt.get().hasTargets()) {
            hubCamHasTarget = false;
            if (prevHubCamHasTarget) {
                ewaCamDist = Double.NaN;
                ewaCamAngle = Double.NaN;
            }
            prevHubCamHasTarget = false;
            return;
        }

        VisionResult result = resultOpt.get();

        // Average target.getYaw() across passing hub tags for turret-relative angle.
        // getYaw() is a direct 2D pixel measurement — robust at any distance, unlike PnP.
        double sumYaw = 0;
        double bestDist = Double.MAX_VALUE; // closest tag's PnP distance (most reliable)
        int count = 0;

        // Reset diagnostic counters for this cycle
        diagTagsSeen = 0;
        diagTagsRejectedAmbiguity = 0;
        diagTagsRejectedNotHub = 0;

        for (VisionTarget target : result.getTargets()) {
            diagTagsSeen++;
            double ambiguity = target.getPoseAmbiguity();
            if (ambiguity >= 0) diagLastAmbiguity = ambiguity;

            if (ambiguity > runtimeAmbiguityThreshold) {
                diagTagsRejectedAmbiguity++;
                continue;
            }

            // Filter: only process hub tags for this alliance
            boolean isHubTag = false;
            for (int id : hubTagIds) {
                if (target.getFiducialId() == id) { isHubTag = true; break; }
            }
            if (!isHubTag) {
                diagTagsRejectedNotHub++;
                continue;
            }

            // Use getYaw() for angle — direct 2D pixel measurement, turret-relative
            sumYaw += target.getYaw();

            // Use closest tag's PnP distance — best accuracy at range, avoids
            // averaging tags at different distances into a composite value
            Transform3d camToTag = target.getBestCameraToTarget();
            double dist = camToTag.getTranslation().getNorm();
            if (dist < bestDist) bestDist = dist;

            count++;
        }

        if (count == 0) {
            hubCamHasTarget = false;
            if (prevHubCamHasTarget) {
                ewaCamDist = Double.NaN;
                ewaCamAngle = Double.NaN;
            }
            prevHubCamHasTarget = false;
            return;
        }

        double rawAngle = sumYaw / count;
        double rawDist = bestDist;

        rawCamDist = rawDist;
        rawCamAngle = rawAngle;

        // EWA: seed on first measurement (NaN sentinel), then blend 4% new / 96% prior
        ewaCamDist = Double.isNaN(ewaCamDist) ? rawDist : ewaCamDist + VisionConstants.kVisionAlpha * (rawDist - ewaCamDist);
        ewaCamAngle = Double.isNaN(ewaCamAngle) ? rawAngle : ewaCamAngle + VisionConstants.kVisionAlpha * (rawAngle - ewaCamAngle);
        hubCamDistanceMeters = ewaCamDist;
        hubCamAngleDegrees = ewaCamAngle;

        // Gate on having ≥1 passing tag — don't use PnP distance for gating
        hubCamHasTarget = true;
        prevHubCamHasTarget = hubCamHasTarget;
    }

    // ==================== Hub Unified Getters (selected approach via Shuffleboard toggle) ====================

    /** Distance to hub center using the active vision mode. */
    public double getHubDistance() {
        if (visionMode == 3) return hubBlendedDistanceMeters + 0.5; // REMOVE ME!!! This is just for temp auton testing on 4/23
        if (visionMode == 2) return hubCamHasTarget ? hubCamDistanceMeters : hubPoseDistanceMeters;
        return visionMode == 0 ? hubCamDistanceMeters : hubPoseDistanceMeters;
    }

    /** Angle to hub center using the active vision mode (degrees, positive = left). */
    public double getHubAngle() {
        if (visionMode == 3) return hubBlendedAngleDegrees;
        if (visionMode == 2) return hubCamHasTarget ? hubCamAngleDegrees : hubPoseAngleDegrees;
        return visionMode == 0 ? hubCamAngleDegrees : hubPoseAngleDegrees;
    }

    /** Whether the hub is visible using the active vision mode. */
    public boolean isHubVisible() {
        if (visionMode == 3) return hubCamHasTarget || hubPoseHasTarget;
        if (visionMode == 2) return hubCamHasTarget || hubPoseHasTarget;
        return visionMode == 0 ? hubCamSticky : hubPoseHasTarget;
    }

    /** Mode-aware freshness: camera-only modes require actual detection per frame;
     *  blended/pose modes treat pose data as always fresh. */
    public boolean isTrackingDataFresh() {
        if (visionMode == 0) return hubCamHasTarget;
        return isHubVisible();
    }

    /** Whether the camera pipeline is online (receiving results, even if no tags visible).
     *  False means USB disconnect, driver crash, or pipeline failure. */
    public boolean isCameraOnline() { return cameraOnline; }

    // ==================== Hub Approach A Getters (camera-only) ====================

    public double getHubCamDistance() { return hubCamDistanceMeters; }
    public double getHubCamAngle() { return hubCamHasTarget ? hubCamAngleDegrees : lastHubCamAngle; }
    /** Sticky visibility — stays true during holdoff after last detection. */
    public boolean isHubCamVisible() { return hubCamSticky; }
    /** Raw per-frame detection — true only when PV actually sees hub tags this frame. */
    public boolean isHubCamFresh() { return hubCamHasTarget; }

    // ==================== Hub Approach B Getters (pose-based) ====================

    public double getHubPoseDistance() { return hubPoseDistanceMeters; }
    public double getHubPoseAngle() { return hubPoseAngleDegrees; }
    public boolean isHubPoseVisible() { return hubPoseHasTarget; }

    // ==================== Hub Shared Getters ====================

    /**
     * Distance to hub center computed purely from robot pose and hardcoded
     * hub coordinates. Independent of camera viewing angle — does not use PnP.
     * Accuracy depends on odometry/vision pose quality.
     */
    public double getPoseBasedDistance() {
        var pose = poseSupplier.get();
        var hub = getHubCenter();
        double dx = hub.getX() - pose.getX();
        double dy = hub.getY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Number of hub tags currently visible. */
    public int getHubVisibleTagCount() { return hubVisibleTagCount; }

    /**
     * @deprecated Moved to {@link com.adambots.utils.FieldGeometry#getHubCenter()} —
     *   hub geometry isn't vision-specific. Callers should switch to FieldGeometry
     *   directly to avoid taking a VisionSubsystem dependency just to read a field
     *   constant. This delegate remains for one release to avoid churn.
     */
    @Deprecated
    public Translation2d getHubCenter() {
        return com.adambots.utils.FieldGeometry.getHubCenter();
    }

    // ==================== Generic Tag Visibility (any tag group) ====================

    /**
     * Checks whether any tag from the given group is currently visible.
     * Works for any tag group — hub, trench, tower, etc.
     */
    public boolean areTagsVisible(int[] tagIds) {
        return photonVision.getVisibleTagCount(tagIds, VisionConstants.kAmbiguityThreshold) > 0;
    }

    // ==================== General Getters ====================

    public String getAllianceColor() {
        if (Utils.isOnRedAlliance()) return "Red";
        if (Utils.isOnBlueAlliance()) return "Blue";
        return "Unknown";
    }

    /** Returns the PhotonVision instance for wiring to swerve. */
    public PhotonVision getPhotonVision() {
        return photonVision;
    }

    // ==================== Hub Triggers ====================

    /** Trigger that fires when the hub is visible (selected approach). */
    public Trigger hubVisibleTrigger() {
        return new Trigger(this::isHubVisible);
    }

    /** Trigger that fires when the hub is visible and within range. */
    public Trigger hubInRangeTrigger(double maxDistanceMeters) {
        return new Trigger(() -> isHubVisible() && getHubDistance() <= maxDistanceMeters);
    }

    /** Trigger that fires when the hub is visible and angle is within tolerance. */
    public Trigger hubAlignedTrigger(double toleranceDegrees) {
        return new Trigger(() -> isHubVisible() && Math.abs(getHubAngle()) <= toleranceDegrees);
    }

    /** Trigger that fires when the hub is visible, in range, AND aligned. */
    public Trigger hubLockedTrigger(double maxDistanceMeters, double toleranceDegrees) {
        return new Trigger(() -> isHubVisible()
            && getHubDistance() <= maxDistanceMeters
            && Math.abs(getHubAngle()) <= toleranceDegrees);
    }

    /** Trigger that fires when any alliance tower tag is visible — useful for climb approach. */
    public Trigger towerVisibleTrigger() {
        return new Trigger(() -> areTagsVisible(
            Utils.isOnRedAlliance() ? VisionConstants.kRedTowerTags : VisionConstants.kBlueTowerTags));
    }

    /** Trigger that fires when any alliance outpost tag is visible. */
    public Trigger outpostVisibleTrigger() {
        return new Trigger(() -> areTagsVisible(
            Utils.isOnRedAlliance() ? VisionConstants.kRedOutpostTags : VisionConstants.kBlueOutpostTags));
    }

    /** Trigger that fires when any alliance trench tag is visible. */
    public Trigger trenchVisibleTrigger() {
        return new Trigger(() -> areTagsVisible(
            Utils.isOnRedAlliance() ? VisionConstants.kRedTrenchTags : VisionConstants.kBlueTrenchTags));
    }

    // ==================== Generic Tag Triggers ====================

    /**
     * Trigger that fires when ANY tag from the given group is visible.
     * Works for any tag group — hub, trench, tower, etc.
     *
     * @param tagIds tag group to check (e.g., VisionConstants.kRedTrenchTags)
     */
    public Trigger tagsVisibleTrigger(int[] tagIds) {
        return new Trigger(() -> areTagsVisible(tagIds));
    }

    // ==================== Hub Commands ====================

    /** Waits until the hub is visible. */
    public Command waitForHubCommand() {
        return Commands.waitUntil(this::isHubVisible);
    }

    /** Waits until aligned to the hub within the given angle tolerance. */
    public Command waitForHubAlignmentCommand(double toleranceDegrees) {
        return Commands.waitUntil(hubAlignedTrigger(toleranceDegrees));
    }

    // ==================== General Commands ====================

    /**
     * Convert robot-relative pose angle to camera-relative offset for blending with camera yaw.
     * The camera faces backward on the turret (180° from turret forward), so we subtract 180°
     * to express the angle relative to the camera boresight — matching the sign convention of
     * PhotonVision's target.getYaw() used in camera-only tracking.
     */
    private double poseAngleToTurretRelative(double poseAngleDeg) {
        double turretOffsetFromForward = turretAngleSupplier.getAsDouble()
            - TurretConstants.kTurretForwardDegrees;
        double raw = poseAngleDeg - turretOffsetFromForward - 180.0;
        // Wrap to [-180, 180] so turret takes the shortest path
        while (raw > 180) raw -= 360;
        while (raw < -180) raw += 360;
        return raw;
    }

    private static final String[] MODE_NAMES = {"Camera", "Pose", "Hybrid", "Blended"};

    /** Logs current vision state — useful for debugging in autos. */
    public Command logVisionCommand() {
        return runOnce(() -> {
            String modeName = (visionMode >= 0 && visionMode <= 3) ? MODE_NAMES[visionMode] : "Unknown";
            System.out.println("[Vision] mode=" + modeName
                + " hubVisible=" + isHubVisible()
                + " hubDist=" + String.format("%.2f", getHubDistance())
                + " hubAngle=" + String.format("%.2f", getHubAngle())
                + " hubTags=" + hubVisibleTagCount
                + " alliance=" + getAllianceColor());
        });
    }
}
