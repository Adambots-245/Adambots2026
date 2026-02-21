package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.RPM;

import com.adambots.Constants.HopperConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * hopper subsystem for feeding balls into the shooter.
 */
public class HopperSubsystem extends SubsystemBase {

    private final BaseMotor hopperMotor;
    private final BaseDistanceSensor hopperPieceSensor;

    public HopperSubsystem(BaseMotor hopperMotor, BaseDistanceSensor hopperPieceSensor) {
        this.hopperMotor = hopperMotor;
        this.hopperPieceSensor = hopperPieceSensor;
        hopperMotor.setInverted(true); // Set to true if motor is reversed
        hopperMotor.setBrakeMode(true);
    }

    public boolean hasPiece(){
      if(hopperPieceSensor.getDistance().in(Centimeters) <= HopperConstants.kDetectionRange){
        return true;
      }

      return false;
    }

    /**
     * Run the hopper at the configured speed.
     */
    public void runHopper() {
        hopperMotor.set(HopperConstants.kHopperSpeed);
    }

    /**
     * Run the hopper in reverse.
     */
    public void reverseHopper() {
        hopperMotor.set(-HopperConstants.kHopperSpeed);
    }

    /**
     * Stop the hopper motor.
     */
    public void stopHopper() {
        hopperMotor.set(0);
    }

    /**
     * Get the hopper motor RPM.
     */
    public double getHopperRPM() {
        return hopperMotor.getVelocity().in(RPM);
    }

    // ==================== Command Factory Methods ====================

    /**
     * Command to run the hopper while held.
     */
    public Command runHopperCommand() {
        return runEnd(this::runHopper, this::stopHopper)
            .withName("Run Hopper");
    }

    /**
     * Command to reverse the hopper while held.
     */
    public Command reverseHopperCommand() {
        return runEnd(this::reverseHopper, this::stopHopper)
            .withName("Reverse Hopper");
    }

    /**
     * Command to stop the hopper (instant).
     */
    public Command stopHopperCommand() {
        return runOnce(this::stopHopper)
            .withName("Stop Hopper");
    }

    public final Trigger hasPiece = new Trigger(() -> hasPiece()); 

   @Override
    public void periodic() {
        // Add telemetry here if needed
    }
}