package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.Dash;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

    // Scan state for oscillating sweep
    private boolean scanningForward = true;

    // Auto-track toggle (driver opts in via Button 5)
    private boolean autoTrackEnabled = false;
    private boolean wasAutoTracking = false;
    private double holdAngleDegrees = 0.0;

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
     * Continuously aims turret at the vision-supplied angle.
     * When target is lost, holds the last known position (PID holds).
     */
    public Command trackHubCommand(DoubleSupplier angleSupplier, BooleanSupplier hasTargetSupplier) {
        return run(() -> {
            if (hasTargetSupplier.getAsBoolean()) {
                setTurretAngle(angleSupplier.getAsDouble());
            }
        }).withName("Track Hub");
    }

    /**
     * Auto-track with three-tier fallback:
     * 1. Camera sees hub → track camera angle (most accurate)
     * 2. Camera lost, pose available → track pose-based angle (keeps roughly aimed)
     * 3. Neither available → oscillating sweep to search for hub
     *
     * Single run() command — evaluates every cycle with no gaps.
     *
     * @param cameraAngle turret-relative angle from shooter camera
     * @param cameraHasTarget whether shooter camera sees the hub
     * @param poseAngle pose-based turret angle (already converted to turret coordinates)
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle) {
        return Commands.runOnce(() -> scanningForward = true)
            .andThen(run(() -> {
                if (!autoTrackEnabled) {
                    if (wasAutoTracking) {
                        holdAngleDegrees = getTurretAngleDegrees();
                        wasAutoTracking = false;
                    }
                    setTurretAngle(holdAngleDegrees);
                    return;
                }
                wasAutoTracking = true;

                if (cameraHasTarget.getAsBoolean()) {
                    // Tier 1: Camera sees hub — most accurate
                    setTurretAngle(cameraAngle.getAsDouble());
                } else {
                    double pose = poseAngle.getAsDouble();
                    if (!Double.isNaN(pose)) {
                        // Tier 2: Pose-based fallback — keeps turret roughly aimed
                        setTurretAngle(pose);
                    } else {
                        // Tier 3: No info — oscillating sweep
                        double target = scanningForward ? TurretConstants.kTurretMaxDegrees : 0.0;
                        setTurretAngle(target);
                        if (isAtTarget(trackingToleranceDeg)) {
                            scanningForward = !scanningForward;
                        }
                    }
                }
            }))
            .withName("Auto Track Hub");
    }
}
