// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for autonomous routines requiring multiple subsystem coordination.
 */
public final class AutoCommands {

    private AutoCommands() {}

    /**
     * Creates a command that shoots preloaded game piece, then drives to a position.
     */
    public static Command scoreAndMoveCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            Pose2d targetPose) {
        return Commands.sequence(
            ShootCommands.shootCommand(shooter, hopper),
            swerve.driveToPoseCommand(targetPose)
        ).withName("ScoreAndMove");
    }
}
