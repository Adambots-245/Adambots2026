// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.adambots.Constants.ClimberConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;
import com.adambots.lib.sensors.LimitSwitch;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        return new Trigger(() -> MathUtil.isNear(ClimberConstants.kLoweredPosition, motor.getPosition(), ClimberConstants.kTolerance));
    }

    /**
     * Returns true when the climber is at the top (extended).
     */
    public Trigger isAtTopTrigger() {
        return new Trigger(() -> limitSwitch.isDetecting());
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
     */
    public Command extendCommand() {
        return run(() -> {
            motor.set(ClimberConstants.kMaxSpeed);
        }).withName("ExtendClimber");
    }

    /**
     * Returns a command that retracts the climber arms (climb action).
     */
    public Command retractCommand() {
        return run(() -> {
            motor.set(BaseMotor.ControlMode.POSITION, ClimberConstants.kLoweredPosition);
        }).withName("RetractClimber");
    }

    /**
     * Returns a command that stops the climber.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            motor.set(0);
            solenoid.enable();
        }).withName("StopClimber");
    }

    /**
     * Returns a command that extends until fully extended, then stops.
     */
    public Command extendFullyCommand() {
        return extendCommand()
            .until(isAtTopTrigger())
            .andThen(stopCommand())
            .withName("ExtendFully");
    }

    /**
     * Returns a command that retracts until at bottom, then stops.
     * Used for initial zeroing or full retraction.
     */
    public Command retractFullyCommand() {
        return retractCommand()
            .until(isAtBottomTrigger())
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
    public Command manualMotorCommand(java.util.function.DoubleSupplier speed) {
        return run(() -> {
            motor.set(speed.getAsDouble());
        }).withName("ManualMotor");
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

        isAtTop = limitSwitch.isDetecting();
        isAtBottom = isAtBottomTrigger().getAsBoolean();
        isClimbing = !(isAtTop || isAtBottom);

        SmartDashboard.putBoolean("Climber/IsAtTop", isAtTop);
        SmartDashboard.putBoolean("Climber/IsAtBottom", isAtBottom);
        SmartDashboard.putBoolean("Climber/IsClimbing", isClimbing);
        SmartDashboard.putNumber("Climber/Position", motor.getPosition());
        SmartDashboard.putNumber("Climber/Solenoid", motor.getPosition());

        Logger.recordOutput("Timing/ClimberSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
    }
}