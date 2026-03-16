package com.adambots.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.lib.utils.Dash;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionCamera;
import com.adambots.lib.vision.VisionCameraInterface;
import com.adambots.lib.vision.VisionResult;
import com.adambots.lib.vision.VisionTarget;
import com.adambots.lib.vision.config.VisionCameraConfig.CameraPurpose;
import com.adambots.lib.vision.config.VisionConfigBuilder;
import com.adambots.lib.vision.config.VisionSystemConfig;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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

    // Median → low-pass filter chains for distance and angle (both approaches)
    private final MedianFilter camDistMedian = new MedianFilter(VisionConstants.kMedianFilterSize);
    private final MedianFilter camAngleMedian = new MedianFilter(VisionConstants.kMedianFilterSize);
    private final MedianFilter poseDistMedian = new MedianFilter(VisionConstants.kMedianFilterSize);
    private final MedianFilter poseAngleMedian = new MedianFilter(VisionConstants.kMedianFilterSize);
    // camDistLowPass removed — low-pass on distance caused 0.19m cold-start bug after filter resets
    private final LinearFilter camAngleLowPass = LinearFilter.singlePoleIIR(
        VisionConstants.kLowPassTimeConstant, VisionConstants.kLoopPeriod);
    private final LinearFilter poseDistLowPass = LinearFilter.singlePoleIIR(
        VisionConstants.kLowPassTimeConstant, VisionConstants.kLoopPeriod);
    private final LinearFilter poseAngleLowPass = LinearFilter.singlePoleIIR(
        VisionConstants.kLowPassTimeConstant, VisionConstants.kLoopPeriod);

    // Runtime vision mode (editable from Shuffleboard): 0=Camera-only, 1=Pose-only, 2=Hybrid
    private int visionMode = VisionConstants.kVisionMode;

    // Tunable ambiguity threshold (default from constants, overridable via TuningManager)
    // Sim camera at 3m+ produces ambiguity 0.7-0.99; real LifeCam is ~0.3-0.8
    private double runtimeAmbiguityThreshold = RobotBase.isSimulation() ? 1.0 : VisionConstants.kAmbiguityThreshold;

    // Diagnostic counters for camera-only target processing
    private int diagTagsSeen = 0;
    private int diagTagsRejectedAmbiguity = 0;
    private int diagTagsRejectedNotHub = 0;
    private double diagLastAmbiguity = -1;

    // Throttled logging (1 Hz)
    private double lastLogTimestamp = 0;

    // ==================== Simulation ====================
    private VisionSystemSim visionSim;
    private PhotonCameraSim shooterCamSim;

    /**
     * Creates a VisionSubsystem with the specified cameras enabled.
     *
     * @param poseSupplier supplies the current robot pose from swerve odometry
     * @param field Field2d for dashboard visualization
     * @param backCamerasEnabled whether back-left and back-right odometry cameras are present
     * @param shooterCameraEnabled whether the shooter alignment camera is present
     */
    public VisionSubsystem(Supplier<Pose2d> poseSupplier, Field2d field,
                           boolean backCamerasEnabled, boolean shooterCameraEnabled) {
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

        if (RobotBase.isSimulation() && hasShooterCamera) {
            setupSimVision();
        }
    }

    private void setupDash() {
        Dash.useTab("Vision");
        int col, row;

        // Row 0: Hub unified outputs (active approach)
        col = 0; row = 0;
        Dash.add("Hub Distance (m)", this::getHubDistance, col++, row);
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

    // ==================== Simulation ====================

    /**
     * Initialize PhotonVision simulation. Creates a VisionSystemSim with a simulated
     * shooter camera that produces synthetic AprilTag detections based on robot pose.
     */
    private void setupSimVision() {
        visionSim = new VisionSystemSim("main");

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        visionSim.addAprilTags(fieldLayout);

        // Configure simulated camera to match the real LifeCam HD-3000
        SimCameraProperties camProps = new SimCameraProperties();
        camProps.setCalibration(
            Constants.SimulationConstants.kCameraResolutionWidth,
            Constants.SimulationConstants.kCameraResolutionHeight,
            Rotation2d.fromDegrees(Constants.SimulationConstants.kCameraFOVDegrees));
        camProps.setCalibError(0.25, 0.08);
        camProps.setFPS(Constants.SimulationConstants.kCameraFPS);
        camProps.setAvgLatencyMs(Constants.SimulationConstants.kCameraAvgLatencyMs);
        camProps.setLatencyStdDevMs(Constants.SimulationConstants.kCameraLatencyStdDevMs);

        // Create simulated camera with the same name as the real one
        PhotonCamera simCam = new PhotonCamera(VisionConstants.kShooterCameraName);
        shooterCamSim = new PhotonCameraSim(simCam, camProps);
        shooterCamSim.enableDrawWireframe(true);

        // Initial static transform (will be updated each cycle for turret rotation)
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                VisionConstants.kShooterCameraX,
                VisionConstants.kShooterCameraY,
                VisionConstants.kShooterCameraZ),
            new Rotation3d(
                0,
                Math.toRadians(-VisionConstants.kShooterCameraPitch),
                Math.toRadians(VisionConstants.kShooterCameraYaw)));
        visionSim.addCamera(shooterCamSim, robotToCamera);

        System.out.println("[Vision] Simulation initialized with shooter camera sim");
    }

    /**
     * Update vision simulation. Must be called from Robot.simulationPeriodic().
     * Rotates the simulated camera transform to match the current turret angle.
     *
     * @param robotPose current robot pose on the field
     * @param turretAngleDeg current turret angle in degrees (0 = forward)
     */
    public void simulationPeriodic(Pose2d robotPose, double turretAngleDeg) {
        if (visionSim == null) return;

        // Update camera transform to rotate with turret
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                VisionConstants.kShooterCameraX,
                VisionConstants.kShooterCameraY,
                VisionConstants.kShooterCameraZ),
            new Rotation3d(
                0,
                Math.toRadians(-VisionConstants.kShooterCameraPitch),
                Math.toRadians(VisionConstants.kShooterCameraTurretOffset - turretAngleDeg)));
        visionSim.adjustCamera(shooterCamSim, robotToCamera);

        visionSim.update(robotPose);
    }

    // ==================== Tunable Setters (called by DashboardSetup) ====================

    public void setVisionMode(int mode) {
        visionMode = hasShooterCamera ? mode : 1;
    }

    public void setAmbiguityThreshold(double threshold) {
        runtimeAmbiguityThreshold = threshold;
    }

    // Raw value getters for DashboardSetup to display behind TUNING_ENABLED
    public double getRawCamDist() { return rawCamDist; }
    public double getRawCamAngle() { return rawCamAngle; }
    public double getRawPoseDist() { return rawPoseDist; }
    public double getRawPoseAngle() { return rawPoseAngle; }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/Mode", visionMode);

        // Pose estimation is handled by SwerveSubsystem.periodic() via swerve.setupVision(vision).
        // Do NOT call updatePoseEstimation() here — PhotonPoseEstimator's timestamp cache
        // would consume the result, preventing swerve from receiving vision measurements.

        // Resolve alliance hub tags and center
        boolean isRed = Utils.isOnRedAlliance();
        Translation2d hubCenter = isRed ? redHubCenter : blueHubCenter;
        int[] hubTagIds = isRed ? VisionConstants.kRedHubTags : VisionConstants.kBlueHubTags;

        // Hub tag count
        hubVisibleTagCount = photonVision.getVisibleTagCount(hubTagIds, VisionConstants.kAmbiguityThreshold);

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
        SmartDashboard.putNumber("Vision/HoldoffCounter", hubCamHoldoffCounter);
        SmartDashboard.putBoolean("Vision/HubSticky", hubCamSticky);
        SmartDashboard.putBoolean("Vision/HubFresh", hubCamHasTarget);

        // ==================== Hub Approach B: Pose-Based ====================
        if (hasBackCameras) {
            Pose2d currentPose = poseSupplier.get();
            // Guard: skip if swerve pose is still at origin (no vision updates processed yet).
            // No camera dependency — Approach B is purely pose-based via swerve odometry.
            if (currentPose.getTranslation().getNorm() > 0) {
                double rawDist = photonVision.getDistanceToPoint(hubCenter);
                double rawAngle = photonVision.getYawToPoint(hubCenter).getDegrees();
                rawPoseDist = rawDist;
                rawPoseAngle = rawAngle;
                // Distance: median → low-pass (safe for linear values)
                hubPoseDistanceMeters = poseDistLowPass.calculate(poseDistMedian.calculate(rawDist));
                // Angle: use raw value — the swerve pose estimator already smooths the pose,
                // and linear filters break at the ±180° wraparound boundary causing wild oscillation.
                hubPoseAngleDegrees = rawAngle;
                hubPoseHasTarget = hubPoseDistanceMeters <= VisionConstants.kMaxDistanceMeters;
            } else {
                hubPoseHasTarget = false;
            }

            // Single reset point: true→false transition (covers both pose-at-origin and out-of-range)
            if (!hubPoseHasTarget && prevHubPoseHasTarget) {
                poseDistMedian.reset();
                poseAngleMedian.reset();
                poseDistLowPass.reset();
                poseAngleLowPass.reset();
            }
            prevHubPoseHasTarget = hubPoseHasTarget;

            // Pose tracking telemetry
            SmartDashboard.putBoolean("Vision/HubPoseVisible", hubPoseHasTarget);
            SmartDashboard.putNumber("Vision/HubPoseTurretAngle", getHubPoseTurretAngle());
        }

        // Throttled diagnostic log (1 Hz) for RioLog copy-paste troubleshooting
        if (Constants.VISION_TAB) {
            double now = Timer.getFPGATimestamp();
            if (now - lastLogTimestamp >= 1.0) {
                lastLogTimestamp = now;
                Pose2d p = poseSupplier.get();
                String trackMode = SmartDashboard.getString("Turret/TrackingMode", "?");
                String modeName = (visionMode >= 0 && visionMode <= 2) ? MODE_NAMES[visionMode] : "?";
                Translation2d hc = isRed ? redHubCenter : blueHubCenter;
                System.out.printf(
                    "[Vision] mode=%s cam=%s(seen=%d ambig=%d id=%d lastA=%.2f) pose=%s dist=%.2f/%.2f ang=%.1f/%.1f track=%s tags=%d pose=(%.1f,%.1f,%.0f°) hub=(%.1f,%.1f)%n",
                    modeName,
                    hubCamHasTarget ? "T" : "F", diagTagsSeen, diagTagsRejectedAmbiguity, diagTagsRejectedNotHub, diagLastAmbiguity,
                    hubPoseHasTarget ? "T" : "F",
                    hubCamDistanceMeters, hubPoseDistanceMeters,
                    hubCamAngleDegrees, hubPoseAngleDegrees,
                    trackMode, hubVisibleTagCount,
                    p.getX(), p.getY(), p.getRotation().getDegrees(),
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
                camDistMedian.reset();
                camAngleMedian.reset();
                camAngleLowPass.reset();
            }
            prevHubCamHasTarget = false;
            return;
        }
        // Trigger results fetch — populates internal resultsList from NetworkTables.
        // The returned pose is ignored (static transform is wrong for turret-mounted camera).
        cam.getEstimatedGlobalPose();
        Optional<? extends VisionResult> resultOpt = cam.getLatestResult();

        if (resultOpt.isEmpty() || !resultOpt.get().hasTargets()) {
            hubCamHasTarget = false;
            if (prevHubCamHasTarget) {
                camDistMedian.reset();
                camAngleMedian.reset();
                camAngleLowPass.reset();
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
                camDistMedian.reset();
                camAngleMedian.reset();
                camAngleLowPass.reset();
            }
            prevHubCamHasTarget = false;
            return;
        }

        double rawAngle = sumYaw / count;
        double rawDist = bestDist;

        rawCamDist = rawDist;
        rawCamAngle = rawAngle;

        // Distance: median only (no low-pass — avoids cold-start after filter reset)
        hubCamDistanceMeters = camDistMedian.calculate(rawDist);
        // Angle: median only — low-pass IIR destroys the signal with intermittent detections
        // (filter starts from 0 each time tags reappear, never reaches true value)
        hubCamAngleDegrees = camAngleMedian.calculate(rawAngle);

        // Gate on having ≥1 passing tag — don't use PnP distance for gating
        hubCamHasTarget = true;
        prevHubCamHasTarget = hubCamHasTarget;
    }

    // ==================== Hub Unified Getters (selected approach via Shuffleboard toggle) ====================

    /** Distance to hub center using the active vision mode. */
    public double getHubDistance() {
        if (visionMode == 2) return hubCamHasTarget ? hubCamDistanceMeters : hubPoseDistanceMeters;
        return visionMode == 0 ? hubCamDistanceMeters : hubPoseDistanceMeters;
    }

    /** Angle to hub center using the active vision mode (degrees, positive = left). */
    public double getHubAngle() {
        if (visionMode == 2) return hubCamHasTarget ? hubCamAngleDegrees : hubPoseAngleDegrees;
        return visionMode == 0 ? hubCamAngleDegrees : hubPoseAngleDegrees;
    }

    /** Whether the hub is visible using the active vision mode. */
    public boolean isHubVisible() {
        if (visionMode == 2) return hubCamHasTarget || hubPoseHasTarget;
        return visionMode == 0 ? hubCamHasTarget : hubPoseHasTarget;
    }

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

    /** Pose-computed turret angle to point at the hub. Clamps to turret travel range.
     *  Sign: getHubPoseAngle() is WPILib CCW-positive (left = positive),
     *  but turret angle increases CW (right = increasing), so we subtract. */
    public double getHubPoseTurretAngle() {
        return MathUtil.clamp(
            TurretConstants.kTurretForwardDegrees - getHubPoseAngle(),
            0, TurretConstants.kTurretMaxDegrees);
    }

    // ==================== Hub Shared Getters ====================

    /** Number of hub tags currently visible. */
    public int getHubVisibleTagCount() { return hubVisibleTagCount; }

    /** Returns the hub center for the current alliance. */
    public Translation2d getHubCenter() {
        return Utils.isOnRedAlliance() ? redHubCenter : blueHubCenter;
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

    private static final String[] MODE_NAMES = {"Camera", "Pose", "Hybrid"};

    /** Logs current vision state — useful for debugging in autos. */
    public Command logVisionCommand() {
        return runOnce(() -> {
            String modeName = (visionMode >= 0 && visionMode <= 2) ? MODE_NAMES[visionMode] : "Unknown";
            System.out.println("[Vision] mode=" + modeName
                + " hubVisible=" + isHubVisible()
                + " hubDist=" + String.format("%.2f", getHubDistance())
                + " hubAngle=" + String.format("%.2f", getHubAngle())
                + " hubTags=" + hubVisibleTagCount
                + " alliance=" + getAllianceColor());
        });
    }
}
