package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Centimeters;

import com.adambots.Constants.HopperConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Hopper subsystem with single motor and CANRange distance sensor for piece detection.
 */
public class HopperSubsystem extends SubsystemBase {

    private final BaseMotor hopperMotor;
    private final BaseDistanceSensor hopperPieceSensor;

    public HopperSubsystem(BaseMotor hopperMotor, BaseDistanceSensor hopperPieceSensor) {
        this.hopperMotor = hopperMotor;
        this.hopperPieceSensor = hopperPieceSensor;
        hopperMotor.setInverted(true);
        hopperMotor.setBrakeMode(true);
    }

    public boolean hasPiece() {
        return hopperPieceSensor.getDistance().in(Centimeters) <= HopperConstants.kDetectionRange;
    }

    public void runHopper() {
        hopperMotor.set(HopperConstants.kHopperSpeed);
    }

    public void reverseHopper() {
        hopperMotor.set(-HopperConstants.kHopperSpeed);
    }

    public void stopHopper() {
        hopperMotor.set(0);
    }

    // ==================== Triggers ====================

    public final Trigger hasPiece = new Trigger(this::hasPiece);

    public Trigger isEmptyTrigger() {
        return new Trigger(() -> !hasPiece());
    }

    // ==================== Command Factories ====================

    public Command runHopperCommand() {
        return runEnd(this::runHopper, this::stopHopper)
            .withName("Run Hopper");
    }

    /** Alias for runHopperCommand() — for readability in ShootCommands. */
    public Command feedCommand() {
        return runHopperCommand();
    }

    public Command reverseHopperCommand() {
        return runEnd(this::reverseHopper, this::stopHopper)
            .withName("Reverse Hopper");
    }

    public Command stopHopperCommand() {
        return runOnce(this::stopHopper)
            .withName("Stop Hopper");
    }

    @Override
    public void periodic() {
    }
}
