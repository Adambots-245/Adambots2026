package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Centimeters;

import com.adambots.Constants.HopperConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Hopper subsystem with hopper + uptake motors and CANRange distance sensor for piece detection.
 * Hopper and uptake always run together, so they share a single subsystem to prevent scheduling conflicts.
 */
public class HopperSubsystem extends SubsystemBase {

    private final BaseMotor hopperMotor;
    private final BaseMotor uptakeMotor;
    private final BaseDistanceSensor hopperPieceSensor;

    public HopperSubsystem(BaseMotor hopperMotor, BaseMotor uptakeMotor, BaseDistanceSensor hopperPieceSensor) {
        this.hopperMotor = hopperMotor;
        this.uptakeMotor = uptakeMotor;
        this.hopperPieceSensor = hopperPieceSensor;
        hopperMotor.setInverted(true);
        hopperMotor.setBrakeMode(true);
        uptakeMotor.setInverted(true);
        uptakeMotor.setBrakeMode(true);
    }

    public boolean hasPiece() {
        if (hopperPieceSensor == null) return false;
        return hopperPieceSensor.getDistance().in(Centimeters) <= HopperConstants.kDetectionRange;
    }

    private void feed() {
        hopperMotor.set(HopperConstants.kHopperSpeed);
        uptakeMotor.set(HopperConstants.kUptakeSpeed);
    }

    private void reverse() {
        hopperMotor.set(-HopperConstants.kHopperSpeed);
        uptakeMotor.set(-HopperConstants.kUptakeSpeed);
    }

    private void stop() {
        hopperMotor.set(0);
        uptakeMotor.set(0);
    }

    // ==================== Triggers ====================

    public final Trigger hasPiece = new Trigger(this::hasPiece);

    public Trigger isEmptyTrigger() {
        return new Trigger(() -> !hasPiece());
    }

    // ==================== Command Factories ====================

    public Command feedCommand() {
        return runEnd(this::feed, this::stop)
            .withName("Feed");
    }

    public Command reverseCommand() {
        return runEnd(this::reverse, this::stop)
            .withName("Reverse Hopper");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Stop Hopper");
    }

    @Override
    public void periodic() {
    }
}
