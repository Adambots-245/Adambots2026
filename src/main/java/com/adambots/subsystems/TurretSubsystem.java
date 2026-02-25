package com.adambots.subsystems;

import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Turret subsystem with position-controlled PID via onboard motor controller.
 * 180° total range centered at 0° (±90°). Hardwired limit switches at each end
 * reset the encoder.
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
        turretMotor.configureSoftLimits(
            TurretConstants.kTurretForwardLimit,
            TurretConstants.kTurretReverseLimit, true);
        turretMotor.setPosition(0);

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
        degrees = MathUtil.clamp(degrees,
            TurretConstants.kTurretMinDegrees, TurretConstants.kTurretMaxDegrees);
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

    /** Reset encoder position — call from limit switch triggers. */
    public void resetEncoder() {
        turretMotor.setPosition(0);
    }

    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(getTurretAngleDegrees() - lastSetpointDegrees) < toleranceDeg;
    }

    // ==================== Triggers ====================

    public Trigger isAtTargetTrigger() {
        return new Trigger(() -> isAtTarget(TurretTrackingConstants.kTrackingToleranceDeg));
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
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
    public Command aimTurretCommand(java.util.function.DoubleSupplier angleSupplier) {
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
}
