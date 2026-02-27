package com.adambots.commands;

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
 * <p>Layout (boolean boxes are 1×1, text/number widgets are 2×1):
 * <pre>
 * Row 0: [Battery(2x1)] [CANivore(2x1)] [RIO CAN(2x1)]
 * Row 1: [IntakeM][IntakeArm][ShootL][ShootR][Turret][HopperM][Uptake][Climber][Ratchet]
 * Row 2: [FL_D][FL_S][FL_E] [FR_D][FR_S][FR_E] [BL_D][BL_S][BL_E] [BR_D][BR_S][BR_E] [Pigeon]
 * </pre>
 */
public class SystemCheckCommand {

    private final CANBus canivoreBus = new CANBus("*");

    public SystemCheckCommand(
            SwerveSubsystem swerve,
            BaseMotor intakeMotor, BaseMotor intakeArmMotor,
            BaseMotor shooterLeftMotor, BaseMotor shooterRightMotor,
            BaseMotor turretMotor,
            BaseMotor hopperMotor, BaseMotor uptakeMotor,
            BaseMotor climberMotor,
            BaseSolenoid ratchetSolenoid) {

        // --- Row 0: System health (2×1 each) ---
        Dash.add("Battery (V)",
            () -> Math.round(RobotController.getBatteryVoltage() * 100.0) / 100.0, 0, 0);
        Dash.add("CANivore", () -> {
            CANBusStatus cs = canivoreBus.getStatus();
            return String.format("%.1f%% Off:%d TX:%d RX:%d",
                cs.BusUtilization * 100.0, cs.BusOffCount, cs.TEC, cs.REC);
        }, 2, 0);
        Dash.add("RIO CAN", () -> {
            CANStatus rs = RobotController.getCANStatus();
            return String.format("%.1f%% Off:%d TX:%d RX:%d",
                rs.percentBusUtilization * 100.0, rs.busOffCount,
                rs.transmitErrorCount, rs.receiveErrorCount);
        }, 4, 0);

        // --- Row 1: Motor health (1×1 boolean boxes) ---
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
            Dash.add(motorNames[i], () -> !m.getMotorType().equals("DummyMotor"), i, 1);
        }
        // Ratchet solenoid — last slot on row 1
        Dash.add("Ratchet", () -> {
            try { ratchetSolenoid.get(); return true; }
            catch (Exception e) { return false; }
        }, motors.length, 1);

        // --- Row 2: Swerve module health (1×1 boolean boxes) ---
        SwerveModule[] modules = swerve.getSwerveDrive().getModules();
        for (int i = 0; i < modules.length && i < 4; i++) {
            final SwerveModule mod = modules[i];
            String label = moduleLabel(mod.configuration.name);
            int base = i * 3;

            // Drive motor — returns valid position
            Dash.add(label + " Drv",
                () -> !Double.isNaN(mod.getDriveMotor().getPosition()), base, 2);
            // Steer motor — returns valid position
            Dash.add(label + " Str",
                () -> !Double.isNaN(mod.getAngleMotor().getPosition()), base + 1, 2);
            // Absolute encoder — no read issue
            Dash.add(label + " Enc",
                () -> !mod.getAbsoluteEncoderReadIssue(), base + 2, 2);
        }
        // Pigeon IMU — heading is not NaN
        Dash.add("Pigeon",
            () -> !Double.isNaN(swerve.getHeading().getDegrees()), 12, 2);
    }

    /** Abbreviate YAGSL module name: "frontleft" → "FL", "backright" → "BR". */
    private static String moduleLabel(String name) {
        if (name.startsWith("front")) return "F" + (name.contains("left") ? "L" : "R");
        if (name.startsWith("back"))  return "B" + (name.contains("left") ? "L" : "R");
        return name.substring(0, Math.min(2, name.length())).toUpperCase();
    }
}
