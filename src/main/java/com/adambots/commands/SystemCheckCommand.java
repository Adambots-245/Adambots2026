package com.adambots.commands;

import com.adambots.Constants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Dash;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import swervelib.SwerveModule;

/**
 * Passive system health dashboard for pit diagnostics.
 *
 * <p>All indicators auto-update every robot loop via Dash suppliers.
 * Construct once during dashboard setup — no need to schedule as a command.
 *
 * <p>Layout flows dynamically using {@code Constants.kShuffleboardCols} to wrap.
 * Occupies rows 0 through {@link #getRowCount()} - 1.
 */
public class SystemCheckCommand {

    private final CANBus canivoreBus = new CANBus("*");
    private int rowCount;

    public SystemCheckCommand(
            SwerveSubsystem swerve,
            BaseMotor intakeMotor, BaseMotor intakeArmMotor,
            BaseMotor shooterLeftMotor, BaseMotor shooterRightMotor,
            BaseMotor turretMotor,
            BaseMotor hopperMotor, BaseMotor uptakeMotor,
            BaseMotor climberMotor,
            BaseSolenoid ratchetSolenoid) {

        int cols = Constants.kShuffleboardCols;
        int col = 0, row = 0;

        // --- System health (2×1 each) ---
        Dash.add("Battery (V)",
            () -> Math.round(RobotController.getBatteryVoltage() * 100.0) / 100.0, col, row);
        col += 2;
        Dash.add("CANivore", () -> {
            CANBusStatus cs = canivoreBus.getStatus();
            return String.format("%.1f%% Off:%d TX:%d RX:%d",
                cs.BusUtilization * 100.0, cs.BusOffCount, cs.TEC, cs.REC);
        }, col, row);
        col += 2;
        Dash.add("RIO CAN", () -> {
            CANStatus rs = RobotController.getCANStatus();
            return String.format("%.1f%% Off:%d TX:%d RX:%d",
                rs.percentBusUtilization * 100.0, rs.busOffCount,
                rs.transmitErrorCount, rs.receiveErrorCount);
        }, col, row);
        col = 0; row++;

        // --- Motor health (1×1 boolean boxes) ---
        BaseMotor[] motors = {
            intakeMotor, intakeArmMotor,
            shooterLeftMotor, shooterRightMotor,
            turretMotor,
            hopperMotor, uptakeMotor,
            climberMotor
        };
        String[] motorNames = {
            "Intake Motor", "Intake Arm",
            "Shooter Left", "Shooter Right",
            "Turret Motor",
            "Hopper Motor", "Uptake",
            "Climber Motor"
        };
        for (int i = 0; i < motors.length; i++) {
            final BaseMotor m = motors[i];
            Dash.add(motorNames[i], () -> !m.getMotorType().equals("DummyMotor"), col, row);
            col++;
            if (col >= cols) { col = 0; row++; }
        }
        // Ratchet solenoid
        Dash.add("Ratchet", () -> {
            try { ratchetSolenoid.get(); return true; }
            catch (Exception e) { return false; }
        }, col, row);
        col = 0; row++;

        // --- Swerve module health (1×1 combined boolean per module) ---
        SwerveModule[] modules = swerve.getSwerveDrive().getModules();
        for (int i = 0; i < modules.length && i < 4; i++) {
            final SwerveModule mod = modules[i];
            String label = moduleLabel(mod.configuration.name);

            Dash.add(label, () ->
                !Double.isNaN(mod.getDriveMotor().getPosition())
                && !Double.isNaN(mod.getAngleMotor().getPosition())
                && !mod.getAbsoluteEncoderReadIssue(), col, row);
            col++;
            if (col >= cols) { col = 0; row++; }
        }
        // Pigeon IMU
        Dash.add("Pigeon",
            () -> !Double.isNaN(swerve.getHeading().getDegrees()), col, row);
        col = 0; row++;

        rowCount = row;
    }

    /** Number of rows occupied by this component's health indicators. */
    public int getRowCount() {
        return rowCount;
    }

    /** Abbreviate YAGSL module name: "frontleft" → "FL", "backright" → "BR". */
    private static String moduleLabel(String name) {
        if (name.startsWith("front")) return "F" + (name.contains("left") ? "L" : "R");
        if (name.startsWith("back"))  return "B" + (name.contains("left") ? "L" : "R");
        return name.substring(0, Math.min(2, name.length())).toUpperCase();
    }
}
