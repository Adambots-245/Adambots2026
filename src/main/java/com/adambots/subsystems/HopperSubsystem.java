// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseProximitySensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

/**
 * Hopper/Indexer subsystem for storing and staging game pieces.
 *
 * <p>This subsystem manages the hopper which consists of:
 * <ul>
 *   <li>Carousel - rotates to store/move game pieces</li>
 *   <li>Uptake - feeds game pieces from the carousel to the shooter</li>
 *   <li>Sensor - detects if the hopper is empty or has game pieces</li>
 * </ul>
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 */
public class HopperSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor carouselMotor;
    private final BaseMotor uptakeMotor;
    private final BaseProximitySensor hopperSensor;

    // ==================== SECTION: STATE ====================
    // TODO: Declare state variables here
    // private boolean hasGamePiece = false;
    // private int gamePieceCount = 0;

    // ==================== SECTION: TRIGGERS ====================
    // Expose state as yes/no questions via Trigger objects

    /**
     * Returns true when a game piece is detected in the hopper.
     */
    public Trigger hasGamePieceTrigger() {
        // TODO: Implement sensor logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the hopper is empty.
     */
    public Trigger isEmptyTrigger() {
        return hasGamePieceTrigger().negate();
    }

    /**
     * Returns true when the hopper is full.
     */
    public Trigger isFullTrigger() {
        // TODO: Implement capacity check
        return new Trigger(() -> false);
    }

    /**
     * Returns true when a game piece is ready to be shot (at uptake position).
     */
    public Trigger isReadyToShootTrigger() {
        // TODO: Implement ready-to-shoot check
        // May need additional sensor or position tracking
        return hasGamePieceTrigger();
    }

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new HopperSubsystem.
     *
     * @param carouselMotor The motor controller for the carousel (passed from RobotContainer)
     * @param uptakeMotor The motor controller for the uptake (passed from RobotContainer)
     * @param hopperSensor The sensor for detecting game pieces (passed from RobotContainer)
     */
    public HopperSubsystem(BaseMotor carouselMotor, BaseMotor uptakeMotor, BaseProximitySensor hopperSensor) {
        this.carouselMotor = carouselMotor;
        this.uptakeMotor = uptakeMotor;
        this.hopperSensor = hopperSensor;

        // TODO: Configure motor settings if needed
        // carouselMotor.setCurrentLimit(30);
        // uptakeMotor.setCurrentLimit(30);
    }

    // ==================== SECTION: COMMAND FACTORIES ====================
    // All subsystem actions should be exposed as commands

    /**
     * Returns a command that runs the carousel to move game pieces.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command runCarouselCommand() {
        return run(() -> {
            // TODO: Set carousel motor speed
        }).withName("RunCarousel");
    }

    /**
     * Returns a command that reverses the carousel.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command reverseCarouselCommand() {
        return run(() -> {
            // TODO: Set carousel motor speed (negative)
        }).withName("ReverseCarousel");
    }

    /**
     * Returns a command that runs the uptake to feed game pieces to the shooter.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command feedCommand() {
        return run(() -> {
            // TODO: Set uptake motor speed to feed forward
        }).withName("Feed");
    }

    /**
     * Returns a command that reverses the uptake (for unjamming).
     * TODO: Set appropriate motor speed in Constants
     */
    public Command reverseUptakeCommand() {
        return run(() -> {
            // TODO: Set uptake motor speed (negative)
        }).withName("ReverseUptake");
    }

    /**
     * Returns a command that stops all hopper motors.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            // TODO: Stop both motors
        }).withName("StopHopper");
    }

    /**
     * Returns a command that stops only the carousel.
     */
    public Command stopCarouselCommand() {
        return runOnce(() -> {
            // TODO: Stop carousel motor
        }).withName("StopCarousel");
    }

    /**
     * Returns a command that stops only the uptake.
     */
    public Command stopUptakeCommand() {
        return runOnce(() -> {
            // TODO: Stop uptake motor
        }).withName("StopUptake");
    }

    /**
     * Returns a command that indexes one game piece into the hopper.
     * Runs the carousel briefly to move game piece into position.
     */
    public Command indexOneCommand() {
        return runCarouselCommand()
            .withTimeout(0.5)
            .andThen(stopCarouselCommand())
            .withName("IndexOne");
    }

    /**
     * Returns a command that runs both carousel and uptake continuously while held.
     */
    public Command continuousFeedCommand() {
        return run(() -> {
            // TODO: Run both motors
            // carouselMotor.set(Constants.HopperConstants.kCarouselSpeed);
            // uptakeMotor.set(Constants.HopperConstants.kUptakeSpeed);
        }).finallyDo(() -> {
            // TODO: Stop both motors when command ends
            // carouselMotor.set(0);
            // uptakeMotor.set(0);
        }).withName("ContinuousFeed");
    }

    // ==================== SECTION: PERIODIC ====================
    /**
     * This method runs every 20ms.
     * Use for: caching sensor values, updating odometry, logging/telemetry.
     * Do NOT use for: control logic, state machines, conditional actions.
     */
    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();

        // TODO: Cache sensor values
        // hasGamePiece = hopperSensor.isDetecting();

        // TODO: Update telemetry
        // SmartDashboard.putBoolean("Hopper/HasGamePiece", hasGamePiece);
        // SmartDashboard.putNumber("Hopper/CarouselVelocity", carouselMotor.getVelocity());
        // SmartDashboard.putNumber("Hopper/UptakeVelocity", uptakeMotor.getVelocity());

        Logger.recordOutput("Timing/HopperSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
    }

    // ==================== SECTION: PRIVATE HELPERS ====================
    // Internal helper methods
}
