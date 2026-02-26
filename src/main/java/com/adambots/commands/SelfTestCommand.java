package com.adambots.commands;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * One-shot pit check: press button, see green/red for every device, plus CAN bus health.
 *
 * <p>Layout — boolean boxes are 1x1, text/number widgets are 2x1:
 * <pre>
 * Row 0: [Run Self-Test(2x1)] [Battery(2x1)] [CANivore(2x1)]    [RIO CAN(2x1)]
 * Row 1: [IntakeMotor][IntakeArm][ShooterL][ShooterR][Turret][HopperM][HopperU][Climber][Ratchet]
 * Row 2: [Hopper Sensor (cm)(2x1)]
 * </pre>
 */
public class SelfTestCommand extends InstantCommand {

    private record DeviceCheck(BaseMotor motor, GenericEntry entry) {}

    private final DeviceCheck[] devices;
    private final BaseSolenoid ratchetSolenoid;
    private final GenericEntry ratchetEntry;

    // System health entries
    private final GenericEntry batteryEntry;
    private final GenericEntry canivoreEntry;
    private final GenericEntry rioCANEntry;

    private final CANBus canivoreBus;

    /**
     * @param tab           the "Self-Test" Shuffleboard tab (widgets are created here)
     * @param startRow      first row available for device boxes (below the button/health row)
     */
    public SelfTestCommand(
            ShuffleboardTab tab, int startRow,
            BaseMotor intakeMotor, BaseMotor intakeArmMotor,
            BaseMotor shooterLeftMotor, BaseMotor shooterRightMotor,
            BaseMotor turretMotor,
            BaseMotor hopperMotor, BaseMotor uptakeMotor,
            BaseMotor climberMotor,
            BaseSolenoid ratchetSolenoid) {

        this.ratchetSolenoid = ratchetSolenoid;
        this.canivoreBus = new CANBus("*");

        // --- System health on row 0 (next to button, 2x1 each) ---
        batteryEntry = tab.add("Battery (V)", 0.0)
            .withPosition(2, 0).withSize(2, 1).getEntry();
        canivoreEntry = tab.add("CANivore", "---")
            .withPosition(4, 0).withSize(2, 1).getEntry();
        rioCANEntry = tab.add("RIO CAN", "---")
            .withPosition(6, 0).withSize(2, 1).getEntry();

        // --- Device boolean boxes: 1x1 each, all on one row ---
        BaseMotor[] motorRefs = {
            intakeMotor, intakeArmMotor,
            shooterLeftMotor, shooterRightMotor,
            turretMotor,
            hopperMotor, uptakeMotor,
            climberMotor,
        };
        String[] names = {
            "Intake Motor", "Intake Arm",
            "Shooter Left", "Shooter Right",
            "Turret Motor",
            "Hopper Motor", "Hopper Uptake",
            "Climber Motor",
        };

        devices = new DeviceCheck[motorRefs.length];
        for (int i = 0; i < motorRefs.length; i++) {
            GenericEntry entry = tab.add(names[i], false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(i, startRow).withSize(1, 1).getEntry();
            devices[i] = new DeviceCheck(motorRefs[i], entry);
        }

        // Ratchet solenoid — next slot after motors on same row
        ratchetEntry = tab.add("Ratchet", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(motorRefs.length, startRow)
            .withSize(1, 1).getEntry();
    }

    @Override
    public void initialize() {
        // Device alive checks
        for (DeviceCheck d : devices) {
            d.entry().setBoolean(!d.motor().getMotorType().equals("DummyMotor"));
        }
        try {
            ratchetSolenoid.get();
            ratchetEntry.setBoolean(true);
        } catch (Exception e) {
            ratchetEntry.setBoolean(false);
        }

        // Battery
        batteryEntry.setDouble(
            Math.round(RobotController.getBatteryVoltage() * 100.0) / 100.0);

        // CANivore bus health (combined into one string)
        CANBusStatus cs = canivoreBus.getStatus();
        canivoreEntry.setString(String.format("%.1f%% Off:%d TX:%d RX:%d",
            cs.BusUtilization * 100.0, cs.BusOffCount, cs.TEC, cs.REC));

        // RoboRIO CAN bus health (combined into one string)
        CANStatus rs = RobotController.getCANStatus();
        rioCANEntry.setString(String.format("%.1f%% Off:%d TX:%d RX:%d",
            rs.percentBusUtilization * 100.0, rs.busOffCount,
            rs.transmitErrorCount, rs.receiveErrorCount));
    }
}
