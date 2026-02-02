// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for drive-related commands using SwerveSubsystem from AdambotsLib.
 *
 * <p>This class provides static factory methods that create commands
 * for driving operations. The SwerveSubsystem itself provides many commands,
 * but this factory is for more complex driving sequences or combinations.
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * Buttons.JoystickButton3.onTrue(DriveCommands.turnToAngleCommand(swerve, 90));
 * </pre>
 *
 * <p>Note: SwerveSubsystem from AdambotsLib already provides:
 * <ul>
 *   <li>driveCommand() - Field-oriented drive with joystick</li>
 *   <li>driveToPoseCommand() - Drive to a specific pose</li>
 *   <li>driveForwardDistanceCommand() - Drive forward a distance</li>
 *   <li>centerModulesCommand() - Center all modules</li>
 *   <li>lock() - X-lock wheels to prevent pushing</li>
 *   <li>zeroGyro() - Reset gyro heading</li>
 *   <li>getAutonomousCommand() - PathPlanner auto routines</li>
 * </ul>
 */
public final class DriveCommands {

    // Prevent instantiation
    private DriveCommands() {}

    /**
     * Creates a command that turns the robot to face a specific angle.
     *
     * @param swerve The swerve drive subsystem
     * @param angleDegrees The target angle in degrees (field-relative)
     * @return Command that turns to the specified angle
     */
    public static Command turnToAngleCommand(SwerveSubsystem swerve, double angleDegrees) {
        return swerve.driveToPoseCommand(
            new Pose2d(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(angleDegrees)
            )
        ).withName("TurnToAngle(" + angleDegrees + ")");
    }

}
