// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;
import com.adambots.lib.sensors.LimitSwitch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

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
@Logged
public class ClimberSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor motor;
    private final BaseSolenoid solenoid;
    private final LimitSwitch limitSwitch;

    // ==================== SECTION: STATE ====================
    private boolean isAtBottom = false;
    private boolean isAtTop = false;
    private boolean isClimbing = false;

    // ==================== SECTION: TRIGGERS ====================
    // Expose state as yes/no questions via Trigger objects

    /**
     * Returns true when the climber is at the bottom (retracted).
     */
    public Trigger isAtBottomTrigger() {
        // TODO: Implement limit switch logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the climber is at the top (extended).
     */
    public Trigger isAtTopTrigger() {
        // TODO: Implement limit switch logic
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the robot is climbing (motors under load).
     */
    public Trigger isClimbingTrigger() {
        return new Trigger(() -> motor.getVelocity().in(RotationsPerSecond) > 0.0);
    }

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new ClimberSubsystem.
     *
     * @param motor The climber motor controller (passed from RobotContainer)
     * @param solenoid The ratchet solenoid (passed from RobotContainer)
     * @param limitSwitch The limit switch at the top of the climber (passed from RobotContainer)
     */
    public ClimberSubsystem(BaseMotor motor, BaseSolenoid solenoid, LimitSwitch limitSwitch) {
        this.motor = motor;
        this.solenoid = solenoid;
        this.limitSwitch = limitSwitch;

        motor.setBrakeMode(true);

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
        double startTime = Timer.getFPGATimestamp();

        // TODO: Cache limit switch values
        // isLeftAtBottom = leftLimitSwitch.isPressed();
        // isRightAtBottom = rightLimitSwitch.isPressed();

        // TODO: Update telemetry
        // SmartDashboard.putBoolean("Climber/LeftAtBottom", isLeftAtBottom);
        // SmartDashboard.putBoolean("Climber/RightAtBottom", isRightAtBottom);
        // SmartDashboard.putNumber("Climber/LeftPosition", leftMotor.getPosition());
        // SmartDashboard.putNumber("Climber/RightPosition", rightMotor.getPosition());

        Logger.recordOutput("Timing/ClimberSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
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
