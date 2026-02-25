package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.adambots.Constants.UptakeConstants;
import com.adambots.lib.actuators.BaseMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Uptake subsystem for feeding balls into the shooter.
 */
public class UptakeSubsystem extends SubsystemBase {

    private final BaseMotor uptakeMotor;

    public UptakeSubsystem(BaseMotor uptakeMotor) {
        this.uptakeMotor = uptakeMotor;
        uptakeMotor.setInverted(true);
        uptakeMotor.setBrakeMode(true);
    }

    public void runUptake() {
        uptakeMotor.set(UptakeConstants.kUptakeSpeed);
    }

    public void reverseUptake() {
        uptakeMotor.set(-UptakeConstants.kUptakeSpeed);
    }

    public void stopUptake() {
        uptakeMotor.set(0);
    }

    public double getUptakeRPM() {
        return uptakeMotor.getVelocity().in(RPM);
    }

    // ==================== Command Factories ====================

    public Command runUptakeCommand() {
        return runEnd(this::runUptake, this::stopUptake)
            .withName("Run Uptake");
    }

    public Command reverseUptakeCommand() {
        return runEnd(this::reverseUptake, this::stopUptake)
            .withName("Reverse Uptake");
    }

    public Command stopUptakeCommand() {
        return runOnce(this::stopUptake)
            .withName("Stop Uptake");
    }

    @Override
    public void periodic() {
    }
}
