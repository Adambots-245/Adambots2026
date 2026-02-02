// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.LimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Climber subsystem for end-game climbing.
 *
 * <p>This subsystem controls the climbing mechanism that allows the robot
 * to climb during the end-game period.
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 */
public class ClimberSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor leftMotor;
    private final BaseMotor rightMotor;
    private final LimitSwitch leftLimitSwitch;
    private final LimitSwitch rightLimitSwitch;

    // ==================== SECTION: STATE ====================
    // TODO: Declare state variables here
    // private boolean isLeftAtBottom = false;
    // private boolean isRightAtBottom = false;
    // private boolean isClimbing = false;

    // ==================== SECTION: TRIGGERS ====================
    // Expose state as yes/no questions via Trigger objects

    /**
     * Returns true when the left climber is at the bottom (retracted).
     */
    public Trigger isLeftAtBottomTrigger() {
        // TODO: Implement limit switch logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the right climber is at the bottom (retracted).
     */
    public Trigger isRightAtBottomTrigger() {
        // TODO: Implement limit switch logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when both climbers are at the bottom (fully retracted).
     */
    public Trigger isFullyRetractedTrigger() {
        // TODO: Implement combined limit switch logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the climber is extended (ready to climb).
     */
    public Trigger isExtendedTrigger() {
        // TODO: Implement position check
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the robot is climbing (motors under load).
     */
    public Trigger isClimbingTrigger() {
        // TODO: Implement current check
        return new Trigger(() -> false);
    }

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new ClimberSubsystem.
     *
     * @param leftMotor The left climber motor controller (passed from RobotContainer)
     * @param rightMotor The right climber motor controller (passed from RobotContainer)
     * @param leftLimitSwitch The left bottom limit switch (passed from RobotContainer)
     * @param rightLimitSwitch The right bottom limit switch (passed from RobotContainer)
     */
    public ClimberSubsystem(BaseMotor leftMotor, BaseMotor rightMotor,
                            LimitSwitch leftLimitSwitch, LimitSwitch rightLimitSwitch) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;

        // TODO: Configure motor settings if needed
        // leftMotor.setCurrentLimit(60);  // Climbers need high current
        // rightMotor.setCurrentLimit(60);
        // leftMotor.setBrakeMode(true);   // Important for climbing!
        // rightMotor.setBrakeMode(true);
    }

    // ==================== SECTION: COMMAND FACTORIES ====================
    // All subsystem actions should be exposed as commands

    /**
     * Returns a command that extends the climber arms upward.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command extendCommand() {
        return run(() -> {
            // TODO: Set motors to extend (positive or negative depends on gearing)
        }).withName("ExtendClimber");
    }

    /**
     * Returns a command that retracts the climber arms (climb action).
     * TODO: Set appropriate motor speed in Constants
     */
    public Command retractCommand() {
        return run(() -> {
            // TODO: Set motors to retract
        }).withName("RetractClimber");
    }

    /**
     * Returns a command that stops the climber.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            // TODO: Stop both motors
        }).withName("StopClimber");
    }

    /**
     * Returns a command that extends until fully extended, then stops.
     */
    public Command extendFullyCommand() {
        return extendCommand()
            .until(isExtendedTrigger())
            .andThen(stopCommand())
            .withName("ExtendFully");
    }

    /**
     * Returns a command that retracts until at bottom, then stops.
     * Used for initial zeroing or full retraction.
     */
    public Command retractFullyCommand() {
        return retractCommand()
            .until(isFullyRetractedTrigger())
            .andThen(stopCommand())
            .withName("RetractFully");
    }

    /**
     * Returns a command that performs the full climb sequence.
     */
    public Command climbCommand() {
        return retractCommand()
            .withName("Climb");
    }

    /**
     * Returns a command that allows manual control of the left climber.
     * For testing or manual adjustment.
     *
     * @param speed Speed supplier (typically from joystick)
     */
    public Command manualLeftCommand(java.util.function.DoubleSupplier speed) {
        return run(() -> {
            // TODO: Set left motor to supplied speed
            // leftMotor.set(speed.getAsDouble());
        }).withName("ManualLeft");
    }

    /**
     * Returns a command that allows manual control of the right climber.
     * For testing or manual adjustment.
     *
     * @param speed Speed supplier (typically from joystick)
     */
    public Command manualRightCommand(java.util.function.DoubleSupplier speed) {
        return run(() -> {
            // TODO: Set right motor to supplied speed
            // rightMotor.set(speed.getAsDouble());
        }).withName("ManualRight");
    }

    // ==================== SECTION: PERIODIC ====================
    /**
     * This method runs every 20ms.
     * Use for: caching sensor values, updating odometry, logging/telemetry.
     * Do NOT use for: control logic, state machines, conditional actions.
     */
    @Override
    public void periodic() {
        // TODO: Cache limit switch values
        // isLeftAtBottom = leftLimitSwitch.isPressed();
        // isRightAtBottom = rightLimitSwitch.isPressed();

        // TODO: Update telemetry
        // SmartDashboard.putBoolean("Climber/LeftAtBottom", isLeftAtBottom);
        // SmartDashboard.putBoolean("Climber/RightAtBottom", isRightAtBottom);
        // SmartDashboard.putNumber("Climber/LeftPosition", leftMotor.getPosition());
        // SmartDashboard.putNumber("Climber/RightPosition", rightMotor.getPosition());
    }

    // ==================== SECTION: PRIVATE HELPERS ====================
    // Internal helper methods

    /**
     * Gets the average climber position.
     *
     * @return The average position of both climbers
     */
    private double getPosition() {
        // TODO: Return average climber position
        // return (leftMotor.getPosition() + rightMotor.getPosition()) / 2.0;
        return 0;
    }
}
