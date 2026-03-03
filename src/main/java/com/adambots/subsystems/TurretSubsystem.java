package com.adambots.subsystems;

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
import edu.wpi.first.wpilibj.Timer;
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

    // Auto-track toggle (driver opts in via Button 5)
    private boolean autoTrackEnabled = false;
    private boolean wasAutoTracking = false;
    private double holdAngleDegrees = 0.0;

    // Tracking tier for diagnostics: 0=idle, 1=camera, 2=pose, 3=smart-scan
    private int trackingTier = 0;

    // Tunable pose-to-turret offset (default 180° = turret 0° faces straight back)
    private GenericEntry poseOffsetEntry;
    private double poseOffsetDegrees = 180.0;

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
            .apply();

        // Hardware limits: both enabled
        // Reverse limit → encoder = 0.0 (home)
        // Forward limit → encoder = maxRotations (120°, so PID stays valid)
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
        if (!Constants.TUNING_ENABLED) return;
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
        poseOffsetEntry = Dash.addTunable("Turret Pose Offset (deg)", 180.0, pos[0], pos[1]);
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
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double rotations = (degrees / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.set(ControlMode.POSITION, rotations);
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
                poseOffsetDegrees = poseOffsetEntry.getDouble(180.0);
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
     * Converts raw vision angles to an absolute turret position, clamped to [0, kTurretMaxDegrees].
     * Camera angle is a turret-relative offset (add to current heading).
     * Pose angle is a robot-relative bearing (subtract poseOffsetDegrees to convert to turret frame).
     * The offset is tunable — default 180° assumes turret 0° faces straight backward.
     */
    private double toAbsoluteTurretAngle(double cameraAngle, boolean cameraHasTarget,
                                          double poseAngle, boolean poseHasTarget) {
        double target;
        if (cameraHasTarget) {
            target = getTurretAngleDegrees() + cameraAngle;
        } else if (poseHasTarget) {
            target = MathUtil.inputModulus(poseAngle - poseOffsetDegrees, 0, 360);
        } else {
            return getTurretAngleDegrees(); // hold position
        }
        return MathUtil.clamp(target, 0, TurretConstants.kTurretMaxDegrees);
    }

    /**
     * Continuously aims turret at the hub using camera and pose vision data.
     * Three-tier fallback:
     * 1. Camera or pose visible → track via toAbsoluteTurretAngle (updates lastPoseIdealAngle when pose available)
     * 2. Both lost → smart scan using lastPoseIdealAngle to aim toward last known hub direction
     */
    public Command trackHubCommand(
            DoubleSupplier cameraAngle, BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle, BooleanSupplier poseHasTarget) {
        return run(() -> {
            boolean camValid = cameraHasTarget.getAsBoolean();
            boolean poseValid = poseHasTarget.getAsBoolean();

            if (camValid || poseValid) {
                // Tier 1/2: Track using best available source
                trackingTier = camValid ? 1 : 2;
                setTurretAngle(toAbsoluteTurretAngle(
                    cameraAngle.getAsDouble(), camValid,
                    poseAngle.getAsDouble(), poseValid));

                // Update lastPoseIdealAngle whenever pose data is available
                if (poseValid) {
                    lastPoseIdealAngle = MathUtil.inputModulus(poseAngle.getAsDouble() - poseOffsetDegrees, 0, 360);
                }
            } else {
                // Tier 3: Both lost — smart scan toward last known hub direction
                trackingTier = 3;
                setTurretAngle(smartScanFallback());
            }
        }).withName("Track Hub");
    }

    /**
     * Uses lastPoseIdealAngle to determine where to aim when both camera and pose are lost.
     * - [0, kTurretMaxDegrees]: hub is within turret range → aim directly at it
     * - (kTurretMaxDegrees, 180]: hub is just past max limit → park at max
     * - (180, 360): hub is past 0° limit (wrapped) → park at 0°
     */
    private double smartScanFallback() {
        if (lastPoseIdealAngle >= 0 && lastPoseIdealAngle <= TurretConstants.kTurretMaxDegrees) {
            // Hub is within turret range — aim directly
            return lastPoseIdealAngle;
        } else if (lastPoseIdealAngle > TurretConstants.kTurretMaxDegrees && lastPoseIdealAngle <= 180) {
            // Hub is just past the max limit — park at max and wait for robot to turn
            return TurretConstants.kTurretMaxDegrees;
        } else {
            // Hub is past the 0° limit (wrapped around) — park at 0° and wait
            return 0;
        }
    }

    /**
     * Auto-track with three-tier fallback:
     * 1. Camera sees hub → track via toAbsoluteTurretAngle (most accurate)
     * 2. Camera lost, pose available → track via toAbsoluteTurretAngle (updates lastPoseIdealAngle)
     * 3. Neither available → smart scan using lastPoseIdealAngle to aim toward last known hub direction
     *
     * Single run() command — evaluates every cycle with no gaps.
     *
     * @param cameraAngle turret-relative offset angle from shooter camera
     * @param cameraHasTarget whether shooter camera sees the hub
     * @param poseAngle robot-relative bearing from pose estimation
     * @param poseHasTarget whether pose data is available
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle,
            BooleanSupplier poseHasTarget) {
        return Commands.runOnce(() -> {
                holdAngleDegrees = getTurretAngleDegrees();
                wasAutoTracking = false;
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

                if (cameraHasTarget.getAsBoolean()) {
                    // Tier 1: Camera sees hub — most accurate
                    trackingTier = 1;
                    setTurretAngle(toAbsoluteTurretAngle(
                        cameraAngle.getAsDouble(), true,
                        poseAngle.getAsDouble(), poseHasTarget.getAsBoolean()));
                } else if (poseHasTarget.getAsBoolean()) {
                    // Tier 2: Pose-based fallback — keeps turret roughly aimed
                    trackingTier = 2;
                    setTurretAngle(toAbsoluteTurretAngle(
                        cameraAngle.getAsDouble(), false,
                        poseAngle.getAsDouble(), true));
                    // Update lastPoseIdealAngle for smart scan fallback
                    lastPoseIdealAngle = MathUtil.inputModulus(poseAngle.getAsDouble() - poseOffsetDegrees, 0, 360);
                } else {
                    // Tier 3: No info — smart scan toward last known hub direction
                    trackingTier = 3;
                    setTurretAngle(smartScanFallback());
                }
            }))
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
        return Commands.runOnce(() -> {
                logTimer[0] = 0;
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
            .until(() -> isAtTarget(trackingToleranceDeg))
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
                double camAng = cameraAngle.getAsDouble();
                double poseAng = poseAngle.getAsDouble();
                double dist = hubDistance.getAsDouble();
                double currentAngle = getTurretAngleDegrees();

                int tier;
                double targetAngle;
                if (camValid) {
                    tier = 1;
                    targetAngle = toAbsoluteTurretAngle(camAng, true, poseAng, poseValid);
                } else if (poseValid) {
                    tier = 2;
                    targetAngle = toAbsoluteTurretAngle(camAng, false, poseAng, true);
                    lastPoseIdealAngle = MathUtil.inputModulus(poseAng - poseOffsetDegrees, 0, 360);
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
