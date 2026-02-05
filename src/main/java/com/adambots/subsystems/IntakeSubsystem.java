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
 * Intake subsystem for acquiring game pieces from the field.
 *
 * <p>This subsystem controls the intake mechanism that picks up game pieces
 * and transfers them to the hopper/indexer.
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 */
public class IntakeSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor intakeMotor;
    private final BaseProximitySensor intakeSensor;

    // ==================== SECTION: STATE ====================
    // TODO: Declare state variables here
    // private boolean hasGamePiece = false;

    // ==================== SECTION: TRIGGERS ====================
    // Expose state as yes/no questions via Trigger objects

    /**
     * Returns true when a game piece is detected in the intake.
     */
    public Trigger hasGamePieceTrigger() {
        // TODO: Implement sensor logic - check if sensor detects object
        // return new Trigger(() -> intakeSensor.isDetected());
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the intake is running.
     */
    public Trigger isRunningTrigger() {
        // TODO: Check if motor is running above threshold
        // return new Trigger(() -> Math.abs(intakeMotor.getVelocity()) > 0.1);
        return new Trigger(() -> false);
    }

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new IntakeSubsystem.
     *
     * @param intakeMotor The motor controller for the intake (passed from RobotContainer)
     * @param intakeSensor The sensor for detecting game pieces (passed from RobotContainer)
     */
    public IntakeSubsystem(BaseMotor intakeMotor, BaseProximitySensor intakeSensor) {
        this.intakeMotor = intakeMotor;
        this.intakeSensor = intakeSensor;

        // TODO: Configure motor settings if needed
        // intakeMotor.setCurrentLimit(40);
        // intakeMotor.setBrakeMode(false);  // Coast for intake
    }

    // ==================== SECTION: COMMAND FACTORIES ====================
    // All subsystem actions should be exposed as commands

    /**
     * Returns a command that runs the intake to pick up game pieces.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command intakeCommand() {
        return run(() -> {
            // TODO: Set intake motor speed
        }).withName("Intake");
    }

    /**
     * Returns a command that runs the intake in reverse to eject game pieces.
     * TODO: Set appropriate motor speed in Constants
     */
    public Command outtakeCommand() {
        return run(() -> {
            // TODO: Set outtake motor speed (negative)
        }).withName("Outtake");
    }

    /**
     * Returns a command that stops the intake.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            // TODO: Stop the motor
        }).withName("StopIntake");
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
        // hasGamePiece = intakeSensor.isDetected();

        // TODO: Update telemetry
        // SmartDashboard.putBoolean("Intake/HasGamePiece", hasGamePiece);
        // SmartDashboard.putNumber("Intake/MotorVelocity", intakeMotor.getVelocity());

        Logger.recordOutput("Timing/IntakeSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
    }

    // ==================== SECTION: PRIVATE HELPERS ====================
    // Internal helper methods
}
