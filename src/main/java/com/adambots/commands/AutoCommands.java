// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for autonomous routines requiring multiple subsystem coordination.
 *
 * <p>This class provides static factory methods that create commands
 * for autonomous operation, coordinating drive, intake, hopper, and shooter.
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * m_autoChooser.addOption("Score and Move", AutoCommands.scoreAndMoveCommand(...));
 * </pre>
 */
public final class AutoCommands {

    // Prevent instantiation
    private AutoCommands() {}

    /**
     * Creates a command that shoots preloaded game piece, then drives to a position.
     *
     * @param swerve The swerve drive subsystem
     * @param shooter The shooter subsystem
     * @param hopper The hopper subsystem
     * @param targetPose The pose to drive to after shooting
     * @return Command for autonomous scoring then moving
     */
    public static Command scoreAndMoveCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            Pose2d targetPose) {
        return Commands.sequence(
            // Shoot preloaded game piece
            ShootCommands.shootCommand(shooter, hopper),
            // Drive to target position
            swerve.driveToPoseCommand(targetPose)
        ).withName("ScoreAndMove");
    }
}
