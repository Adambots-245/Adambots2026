package com.adambots.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Turret subsystem with position-controlled PID via onboard motor controller.
 * Hardware limit switches are hardwired to the TalonFXS (Minion motor).
 * The reverse limit auto-zeros the encoder — pre-place turret against it before each match.
 * Forward limit records the discovered max range dynamically.
 */
public class TurretSubsystem extends SubsystemBase {

    private final BaseMotor turretMotor;

    // Tunable PID entries
    private GenericEntry turretPEntry;
    private GenericEntry turretIEntry;
    private GenericEntry turretDEntry;
    private double lastTurretP, lastTurretI, lastTurretD;

    // Track last setpoint for isAtTarget()
    private double lastSetpointDegrees = 0;

    // Track angle each cycle so we can capture it before a limit-switch encoder reset
    private double lastKnownAngleDegrees = 0;

    // Calibration state
    private boolean isCalibrated = false;
    private double maxDiscoveredDegrees = TurretConstants.kTurretMaxDegrees;

    // Scan state for oscillating sweep
    private boolean scanningForward = true;

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

        // Hardware limits: both enabled, both auto-reset encoder to 0 when hit
        turretMotor.configureHardLimits(true, true, 0.0, 0.0);

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
        turretPEntry = Dash.addTunable("Turret kP", TurretConstants.kTurretP, pos[0], pos[1]);
        advance(pos, cols);
        turretIEntry = Dash.addTunable("Turret kI", TurretConstants.kTurretI, pos[0], pos[1]);
        advance(pos, cols);
        turretDEntry = Dash.addTunable("Turret kD", TurretConstants.kTurretD, pos[0], pos[1]);
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
        return new Trigger(() -> isAtTarget(TurretTrackingConstants.kTrackingToleranceDeg));
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        // Check hardware limit switches for calibration and range discovery
        if (turretMotor.getReverseLimitSwitch()) {
            isCalibrated = true;
        }
        if (turretMotor.getForwardLimitSwitch()) {
            // Encoder just got reset to 0 by the limit switch — use pre-reset angle
            if (lastKnownAngleDegrees > 0) {
                maxDiscoveredDegrees = lastKnownAngleDegrees;
            }
        }

        // Track angle every cycle so we have the pre-reset value when a limit triggers
        lastKnownAngleDegrees = getTurretAngleDegrees();

        // Dashboard telemetry
        SmartDashboard.putNumber("Turret/Angle (deg)", getTurretAngleDegrees());
        SmartDashboard.putBoolean("Turret/Calibrated", isCalibrated);
        SmartDashboard.putNumber("Turret/Max Range (deg)", maxDiscoveredDegrees);

        // Hot-reload PID from Shuffleboard tunables
        if (turretPEntry != null) {
            double p = turretPEntry.getDouble(TurretConstants.kTurretP);
            double i = turretIEntry.getDouble(TurretConstants.kTurretI);
            double d = turretDEntry.getDouble(TurretConstants.kTurretD);

            if (p != lastTurretP || i != lastTurretI || d != lastTurretD) {
                turretMotor.setPID(0, p, i, d, TurretConstants.kTurretFF);
                lastTurretP = p;
                lastTurretI = i;
                lastTurretD = d;
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
                setTurretAngle(0);
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
     * Position-controlled oscillating scan between 0° and maxDiscoveredDegrees.
     * Uses setTurretAngle (position control) for smooth deceleration at endpoints.
     */
    public Command scanForHubCommand() {
        return Commands.runOnce(() -> scanningForward = true)
            .andThen(run(() -> {
                double target = scanningForward ? maxDiscoveredDegrees : 0.0;
                setTurretAngle(target);
                if (isAtTarget(TurretTrackingConstants.kTrackingToleranceDeg)) {
                    scanningForward = !scanningForward;
                }
            }))
            .withName("Scan For Hub");
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
                        double target = scanningForward ? maxDiscoveredDegrees : 0.0;
                        setTurretAngle(target);
                        if (isAtTarget(TurretTrackingConstants.kTrackingToleranceDeg)) {
                            scanningForward = !scanningForward;
                        }
                    }
                }
            }))
            .withName("Auto Track Hub");
    }
}
