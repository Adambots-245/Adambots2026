package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.Dash;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Turret subsystem with position-controlled PID via onboard motor controller.
 * Hardware limit switches are hardwired to the TalonFXS (Minion motor).
 * The reverse limit auto-zeros the encoder — pre-place turret against it before each match.
 * Forward limit records the discovered max range dynamically.
 */
@Logged
public class TurretSubsystem extends SubsystemBase {

    private final BaseMotor turretMotor;

    // Tunable PID entries
    private GenericEntry turretPEntry;
    private GenericEntry turretIEntry;
    private GenericEntry turretDEntry;
    private GenericEntry turretFFEntry;
    private GenericEntry trackingToleranceEntry;
    private double lastTurretP, lastTurretI, lastTurretD;
    private double lastTurretFF = TurretConstants.kTurretFF;
    private double trackingToleranceDeg = TurretTrackingConstants.kTrackingToleranceDeg;

    // Track last setpoint for isAtTarget()
    private double lastSetpointDegrees = 0;

    // Calibration state
    private boolean isCalibrated = false;

    // Last known pose-based ideal angle for smart scan fallback (default: center of range)
    private double lastPoseIdealAngle = TurretConstants.kTurretMaxDegrees / 2.0;

    // Scan direction for camera-only tracking: +1 = toward max, -1 = toward 0
    private int scanDirection = 1;

    // Auto-track toggle (driver opts in via Button 5)
    private boolean autoTrackEnabled = false;
    private boolean wasAutoTracking = false;
    private double holdAngleDegrees = 0.0;

    // Tracking tier for diagnostics: 0=idle, 1=camera, 2=pose, 3=smart-scan
    private int trackingTier = 0;

    // Camera hysteresis: require N consecutive valid frames before switching to camera tier
    private int camValidFrames = 0;

    // Pose-to-turret offset: poseAngle - offset = turret angle.
    // Derived from kTurretForwardDegrees (the turret angle that faces robot forward).
    private GenericEntry poseOffsetEntry;
    private double poseOffsetDegrees = 360.0 - TurretConstants.kTurretForwardDegrees;

    public TurretSubsystem(BaseMotor turretMotor) {
        this.turretMotor = turretMotor;
        configureMotors();
    }

    private void configureMotors() {
        turretMotor.configure()
            .pid(TurretConstants.kTurretP, TurretConstants.kTurretI,
                 TurretConstants.kTurretD, TurretConstants.kTurretFF)
            .brakeMode(true)
            .currentLimits(TurretConstants.kTurretStallCurrentLimit,
                           TurretConstants.kTurretFreeCurrentLimit, 3000)
            .motionMagic(
                RotationsPerSecond.of(TurretConstants.kTurretCruiseVelocity),
                RotationsPerSecondPerSecond.of(TurretConstants.kTurretAcceleration),
                TurretConstants.kTurretJerk)
            .apply();

        // Hardware limits: both enabled
        // Reverse limit → encoder = 0.0 (home)
        // Forward limit → encoder = maxRotations (kTurretMaxDegrees, so PID stays valid)
        double maxRotations = (TurretConstants.kTurretMaxDegrees / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.configureHardLimits(true, true, maxRotations, 0.0);

        lastTurretP = TurretConstants.kTurretP;
        lastTurretI = TurretConstants.kTurretI;
        lastTurretD = TurretConstants.kTurretD;
    }

    // ==================== Tunable Setup ====================

    /**
     * Registers turret tunable GenericEntry fields on the current Dash tab.
     * Call after Dash.useTab() in RobotContainer.
     */
    public void setupTurretTunables(int[] pos, int cols) {
        if (!Constants.SHOOTER_TAB) return;
        turretPEntry = Dash.addTunable("Turret kP", TurretConstants.kTurretP, pos[0], pos[1]);
        advance(pos, cols);
        turretIEntry = Dash.addTunable("Turret kI", TurretConstants.kTurretI, pos[0], pos[1]);
        advance(pos, cols);
        turretDEntry = Dash.addTunable("Turret kD", TurretConstants.kTurretD, pos[0], pos[1]);
        advance(pos, cols);
        turretFFEntry = Dash.addTunable("Turret kF", TurretConstants.kTurretFF, pos[0], pos[1]);
        advance(pos, cols);
        trackingToleranceEntry = Dash.addTunable("Track Tol (deg)", TurretTrackingConstants.kTrackingToleranceDeg, pos[0], pos[1]);
        advance(pos, cols);
        poseOffsetEntry = Dash.addTunable("Turret Pose Offset (deg)", 360.0 - TurretConstants.kTurretForwardDegrees, pos[0], pos[1]);
        advance(pos, cols);
        Dash.add("Forward Limit", ()->turretMotor.getForwardLimitSwitch(), pos[0], pos[1]);
        advance(pos, cols);
        Dash.add("Reverse Limit", ()->turretMotor.getReverseLimitSwitch(), pos[0], pos[1]);
        advance(pos, cols);
    }

    private static void advance(int[] pos, int cols) {
        pos[0]++;
        if (pos[0] >= cols) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    // ==================== Turret Control ====================

    public void setTurretAngle(double degrees) {
        if (!isCalibrated) return;
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double rotations = (degrees / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.set(ControlMode.MOTION_MAGIC, rotations);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public double getTurretAngleDegrees() {
        return (turretMotor.getPosition() / TurretConstants.kTurretGearRatio) * 360.0;
    }

    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(getTurretAngleDegrees() - lastSetpointDegrees) < toleranceDeg;
    }

    // ==================== Calibration State ====================

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public Trigger isCalibratedTrigger() {
        return new Trigger(this::isCalibrated);
    }

    // ==================== Triggers ====================

    public Trigger isAtTargetTrigger() {
        return new Trigger(() -> isAtTarget(trackingToleranceDeg));
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        // Check hardware limit switches for calibration
        if (turretMotor.getReverseLimitSwitch()) {
            isCalibrated = true;
        }

        // Dashboard telemetry
        SmartDashboard.putNumber("Turret/Angle (deg)", getTurretAngleDegrees());
        SmartDashboard.putBoolean("Turret/Calibrated", isCalibrated);
        SmartDashboard.putBoolean("Turret/AutoTrack", autoTrackEnabled);
        SmartDashboard.putNumber("Turret/Setpoint (deg)", lastSetpointDegrees);
        SmartDashboard.putNumber("Turret/Error (deg)", lastSetpointDegrees - getTurretAngleDegrees());
        SmartDashboard.putNumber("Turret/TrackingTier", trackingTier);
        // Hot-reload PID from Shuffleboard tunables
        if (turretPEntry != null) {
            double p = turretPEntry.getDouble(TurretConstants.kTurretP);
            double i = turretIEntry.getDouble(TurretConstants.kTurretI);
            double d = turretDEntry.getDouble(TurretConstants.kTurretD);

            double f = turretFFEntry != null
                ? turretFFEntry.getDouble(TurretConstants.kTurretFF) : TurretConstants.kTurretFF;

            if (p != lastTurretP || i != lastTurretI || d != lastTurretD || f != lastTurretFF) {
                turretMotor.setPID(0, p, i, d, f);
                lastTurretP = p;
                lastTurretI = i;
                lastTurretD = d;
                lastTurretFF = f;
            }

            if (trackingToleranceEntry != null) {
                trackingToleranceDeg = trackingToleranceEntry.getDouble(TurretTrackingConstants.kTrackingToleranceDeg);
            }
            if (poseOffsetEntry != null) {
                poseOffsetDegrees = poseOffsetEntry.getDouble(360.0 - TurretConstants.kTurretForwardDegrees);
            }
        }
    }

    // ==================== Command Factories ====================

    public Command aimTurretCommand(double degrees) {
        return Commands.runOnce(() -> setTurretAngle(degrees))
            .withName("Turret " + degrees + " deg");
    }

    /** Continuously aims turret at angle from supplier (for vision tracking). */
    public Command aimTurretCommand(DoubleSupplier angleSupplier) {
        return run(() -> setTurretAngle(angleSupplier.getAsDouble()))
            .withName("Aim Turret Dynamic");
    }

    /** Slowly sweeps turret at scan speed. Stops when command ends. */
    public Command scanCommand(double speed) {
        return runEnd(
            () -> turretMotor.set(speed),
            this::stopTurret
        ).withName("Scan Turret");
    }

    /** Holds current turret angle. */
    public Command holdPositionCommand() {
        return runOnce(() -> setTurretAngle(getTurretAngleDegrees()))
            .withName("Hold Position");
    }

    public Command stopTurretCommand() {
        return Commands.runOnce(this::stopTurret, this)
            .withName("Stop Turret");
    }

    public Command toggleAutoTrackCommand() {
        return Commands.runOnce(() -> autoTrackEnabled = !autoTrackEnabled)
            .withName("Toggle Auto-Track");
    }

    // ==================== Calibration Command ====================

    /**
     * Backup calibration: slowly drives toward reverse limit.
     * Hardware auto-zeros encoder when reverse limit is hit.
     * Not wired by default — use if pre-placing isn't reliable.
     */
    public Command calibrateCommand() {
        return runEnd(
            () -> turretMotor.set(-TurretConstants.kCalibrationSpeed),
            this::stopTurret
        )
        .until(turretMotor::getReverseLimitSwitch)
        .withTimeout(TurretConstants.kCalibrationTimeoutSec)
        .andThen(Commands.runOnce(() -> {
            if (isCalibrated) {
                // Move slightly off the reverse limit to avoid PID fighting the hard stop
                setTurretAngle(TurretConstants.kCalibrationOffsetDegrees);
            }
        }, this))
        .withName("Calibrate Turret");
    }

    // ==================== Vision Tracking Commands ====================

    /**
     * Converts a robot-relative pose bearing to an unclamped turret angle.
     * Uses centered inputModulus so that bearings outside the turret's arc
     * map to the nearest limit (0° or kTurretMaxDegrees) after clamping,
     * rather than always wrapping to the far limit.
     */
    private double poseAngleToTurretAngle(double poseAngle) {
        double turretCenterBearing = poseOffsetDegrees + TurretConstants.kTurretMaxDegrees / 2.0;
        return MathUtil.inputModulus(poseAngle - turretCenterBearing, -180, 180)
               + TurretConstants.kTurretMaxDegrees / 2.0;
    }

    /**
     * Converts raw vision angles to an absolute turret position, clamped to [0, kTurretMaxDegrees].
     * Camera angle is a turret-relative offset (add to current heading).
     * Pose angle is a robot-relative bearing (subtract poseOffsetDegrees to convert to turret frame).
     * The offset is tunable — derived from kTurretForwardDegrees so turret center faces robot forward.
     */
    private double toAbsoluteTurretAngle(double cameraAngle, boolean cameraHasTarget,
                                          double poseAngle, boolean poseHasTarget) {
        double target;
        if (cameraHasTarget) {
            target = getTurretAngleDegrees() + cameraAngle;
        } else if (poseHasTarget) {
            target = poseAngleToTurretAngle(poseAngle);
        } else {
            return getTurretAngleDegrees(); // hold position
        }
        return MathUtil.clamp(target, 0, TurretConstants.kTurretMaxDegrees);
    }

    /**
     * Continuously aims turret at the hub using camera-only scan-and-track.
     * TRACKING: camera sees hub → aim directly using camera offset.
     * SCANNING: camera lost → sweep turret in last-known direction, reversing at limits.
     */
    public Command trackHubCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget) {
        return run(() -> {
            boolean camValid = cameraHasTarget.getAsBoolean();
            camValidFrames = camValid ? camValidFrames + 1 : 0;
            boolean useCam = camValid && camValidFrames >= TurretTrackingConstants.kCamHysteresisFrames;

            if (useCam) {
                trackingTier = 1;
                double camAng = cameraAngle.getAsDouble();
                double target = MathUtil.clamp(
                    getTurretAngleDegrees() + camAng,
                    0, TurretConstants.kTurretMaxDegrees);
                setTurretAngle(target);
                scanDirection = (camAng >= 0) ? 1 : -1;
            } else {
                trackingTier = 3;
                if (getTurretAngleDegrees() >= TurretConstants.kTurretMaxDegrees - 1.0) scanDirection = -1;
                else if (getTurretAngleDegrees() <= 1.0) scanDirection = 1;
                turretMotor.set(scanDirection * TurretTrackingConstants.kScanSpeed);
            }
        }).finallyDo(interrupted -> stopTurret())
          .withName("Track Hub");
    }

    /**
     * Uses lastPoseIdealAngle to determine where to aim when both camera and pose are lost.
     * lastPoseIdealAngle is an unclamped turret angle from poseAngleToTurretAngle():
     * - [0, kTurretMaxDegrees]: hub is within turret range → aim directly
     * - > kTurretMaxDegrees: hub is past max limit → park at max
     * - < 0: hub is past min limit → park at 0
     */
    private double smartScanFallback() {
        if (lastPoseIdealAngle >= 0 && lastPoseIdealAngle <= TurretConstants.kTurretMaxDegrees) {
            return lastPoseIdealAngle;
        } else if (lastPoseIdealAngle > TurretConstants.kTurretMaxDegrees) {
            return TurretConstants.kTurretMaxDegrees;
        } else {
            return 0;
        }
    }

    /**
     * Auto-track with camera-only scan-and-track.
     * TRACKING: camera sees hub → aim directly using camera offset.
     * SCANNING: camera lost → sweep turret in last-known direction, reversing at limits.
     * Respects autoTrackEnabled toggle — holds position when disabled.
     *
     * @param cameraAngle turret-relative offset angle from shooter camera
     * @param cameraHasTarget whether shooter camera sees the hub
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget) {
        return Commands.runOnce(() -> {
                holdAngleDegrees = getTurretAngleDegrees();
                wasAutoTracking = false;
                camValidFrames = 0;
            }, this)
            .andThen(run(() -> {
                if (!autoTrackEnabled) {
                    if (wasAutoTracking) {
                        holdAngleDegrees = getTurretAngleDegrees();
                        wasAutoTracking = false;
                    }
                    trackingTier = 0;
                    setTurretAngle(holdAngleDegrees);
                    return;
                }
                wasAutoTracking = true;

                boolean camValid = cameraHasTarget.getAsBoolean();
                camValidFrames = camValid ? camValidFrames + 1 : 0;
                boolean useCam = camValid && camValidFrames >= TurretTrackingConstants.kCamHysteresisFrames;

                if (useCam) {
                    // TRACKING: camera has hub — aim directly
                    trackingTier = 1;
                    double camAng = cameraAngle.getAsDouble();
                    double target = MathUtil.clamp(
                        getTurretAngleDegrees() + camAng,
                        0, TurretConstants.kTurretMaxDegrees);
                    setTurretAngle(target);

                    // Remember which way hub is for scanning if lost
                    scanDirection = (camAng >= 0) ? 1 : -1;
                } else {
                    // SCANNING: sweep to find hub
                    trackingTier = 3;
                    double current = getTurretAngleDegrees();

                    // Reverse at limits
                    if (current >= TurretConstants.kTurretMaxDegrees - 1.0) {
                        scanDirection = -1;
                    } else if (current <= 1.0) {
                        scanDirection = 1;
                    }

                    turretMotor.set(scanDirection * TurretTrackingConstants.kScanSpeed);
                }
            }))
            .finallyDo(interrupted -> stopTurret())
            .withName("Auto Track Hub");
    }

    // ==================== Diagnostic Commands ====================

    /**
     * Diagnostic: one-shot align to hub with dense logging.
     * Logs at 5 Hz showing every step of the angle computation.
     * Runs until turret is on-target (within tolerance) or 8-second timeout.
     * Use: point camera roughly at hub, press button, copy RioLog output.
     */
    public Command diagAlignCommand(
            DoubleSupplier cameraAngle, BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle, BooleanSupplier poseHasTarget,
            DoubleSupplier hubDistance) {
        final double[] logTimer = {0};
        final boolean[] hasExecuted = {false};
        return Commands.runOnce(() -> {
                logTimer[0] = 0;
                hasExecuted[0] = false;
                System.out.println("[DiagAlign] === START === turret=" + String.format("%.1f", getTurretAngleDegrees())
                    + "° poseOffset=" + String.format("%.1f", poseOffsetDegrees) + "°");
            }, this)
            .andThen(run(() -> {
                boolean camValid = cameraHasTarget.getAsBoolean();
                boolean poseValid = poseHasTarget.getAsBoolean();
                double camAng = cameraAngle.getAsDouble();
                double poseAng = poseAngle.getAsDouble();
                double dist = hubDistance.getAsDouble();
                double currentAngle = getTurretAngleDegrees();

                double targetAngle = toAbsoluteTurretAngle(camAng, camValid, poseAng, poseValid);
                setTurretAngle(targetAngle);
                hasExecuted[0] = true;

                // Log at 5 Hz (every 0.2s)
                logTimer[0] += 0.02;
                if (logTimer[0] >= 0.2) {
                    logTimer[0] = 0;
                    String src = camValid ? "CAM" : poseValid ? "POSE" : "NONE";
                    double rawInput = camValid ? camAng : poseAng;
                    System.out.printf(
                        "[DiagAlign] src=%s rawAng=%.1f camAng=%.1f poseAng=%.1f turret=%.1f→%.1f err=%.1f dist=%.2f%n",
                        src, rawInput, camAng, poseAng, currentAngle, targetAngle,
                        targetAngle - currentAngle, dist);
                }
            }))
            .until(() -> hasExecuted[0] && isAtTarget(trackingToleranceDeg))
            .withTimeout(8.0)
            .finallyDo(interrupted -> {
                System.out.printf("[DiagAlign] === %s === turret=%.1f° setpoint=%.1f° err=%.1f°%n",
                    interrupted ? "TIMEOUT" : "ALIGNED",
                    getTurretAngleDegrees(), lastSetpointDegrees,
                    lastSetpointDegrees - getTurretAngleDegrees());
            })
            .withName("Diag: Align");
    }

    /**
     * Diagnostic: continuous auto-track with dense logging.
     * Logs at 4 Hz showing tier transitions, angle sources, and turret movement.
     * Hold the button / run command while moving the robot around, then release.
     * Copy RioLog output for analysis.
     */
    public Command diagAutoTrackCommand(
            DoubleSupplier cameraAngle, BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle, BooleanSupplier poseHasTarget,
            DoubleSupplier hubDistance) {
        final double[] logTimer = {0};
        final int[] prevTier = {-1};
        return Commands.runOnce(() -> {
                logTimer[0] = 0;
                prevTier[0] = -1;
                System.out.println("[DiagTrack] === START === turret=" + String.format("%.1f", getTurretAngleDegrees())
                    + "° poseOffset=" + String.format("%.1f", poseOffsetDegrees) + "°");
            }, this)
            .andThen(run(() -> {
                boolean camValid = cameraHasTarget.getAsBoolean();
                boolean poseValid = poseHasTarget.getAsBoolean();
                camValidFrames = camValid ? camValidFrames + 1 : 0;
                boolean useCam = camValid && camValidFrames >= TurretTrackingConstants.kCamHysteresisFrames;
                double camAng = cameraAngle.getAsDouble();
                double poseAng = poseAngle.getAsDouble();
                double dist = hubDistance.getAsDouble();
                double currentAngle = getTurretAngleDegrees();

                int tier;
                double targetAngle;
                if (useCam) {
                    tier = 1;
                    targetAngle = toAbsoluteTurretAngle(camAng, true, poseAng, poseValid);
                } else if (poseValid) {
                    tier = 2;
                    targetAngle = toAbsoluteTurretAngle(camAng, false, poseAng, true);
                    lastPoseIdealAngle = poseAngleToTurretAngle(poseAng);
                } else {
                    tier = 3;
                    targetAngle = smartScanFallback();
                }
                trackingTier = tier;
                setTurretAngle(targetAngle);

                // Log tier transitions immediately
                if (tier != prevTier[0]) {
                    System.out.printf("[DiagTrack] TIER %d→%d turret=%.1f° target=%.1f°%n",
                        prevTier[0], tier, currentAngle, targetAngle);
                    prevTier[0] = tier;
                }

                // Regular log at 4 Hz (every 0.25s)
                logTimer[0] += 0.02;
                if (logTimer[0] >= 0.25) {
                    logTimer[0] = 0;
                    String src = tier == 1 ? "CAM" : tier == 2 ? "POSE" : "SCAN";
                    System.out.printf(
                        "[DiagTrack] t=%s camAng=%.1f poseAng=%.1f turret=%.1f→%.1f err=%.1f dist=%.2f%n",
                        src, camAng, poseAng, currentAngle, targetAngle,
                        targetAngle - currentAngle, dist);
                }
            }))
            .finallyDo(interrupted -> {
                System.out.printf("[DiagTrack] === STOP === turret=%.1f° lastTier=%d%n",
                    getTurretAngleDegrees(), trackingTier);
            })
            .withName("Diag: Auto Track");
    }
}
