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
        uptakeMotor.setInverted(true); // Set to true if motor is reversed
        uptakeMotor.setBrakeMode(true);
    }

    /**
     * Run the uptake at the configured speed.
     */
    public void runUptake() {
        uptakeMotor.set(UptakeConstants.kUptakeSpeed);
    }

    /**
     * Run the uptake in reverse.
     */
    public void reverseUptake() {
        uptakeMotor.set(-UptakeConstants.kUptakeSpeed);
    }

    /**
     * Stop the uptake motor.
     */
    public void stopUptake() {
        uptakeMotor.set(0);
    }

    /**
     * Get the uptake motor RPM.
     */
    public double getUptakeRPM() {
        return uptakeMotor.getVelocity().in(RPM);
    }

    // ==================== Command Factory Methods ====================

    /**
     * Command to run the uptake while held.
     */
    public Command runUptakeCommand() {
        return runEnd(this::runUptake, this::stopUptake)
            .withName("Run Uptake");
    }

    /**
     * Command to reverse the uptake while held.
     */
    public Command reverseUptakeCommand() {
        return runEnd(this::reverseUptake, this::stopUptake)
            .withName("Reverse Uptake");
    }

    /**
     * Command to stop the uptake (instant).
     */
    public Command stopUptakeCommand() {
        return runOnce(this::stopUptake)
            .withName("Stop Uptake");
    }

    @Override
    public void periodic() {
        // Add telemetry here if needed
    }
}