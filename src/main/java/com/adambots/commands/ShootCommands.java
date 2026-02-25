// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for coordinating shooter and hopper subsystems.
 *
 * <p>This class provides static factory methods that create commands
 * requiring coordination between the shooter and hopper.
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * Buttons.XboxRightBumper.onTrue(ShootCommands.shootCommand(m_shooter, m_hopper));
 * </pre>
 */
public final class ShootCommands {

    // Prevent instantiation
    private ShootCommands() {}

    /**
     * Creates a command that shoots a game piece.
     * Spins up the shooter, waits until at speed, then feeds from hopper.
     *
     * @param shooter The shooter subsystem
     * @param hopper The hopper subsystem
     * @return Command that shoots one game piece
     */
    public static Command shootCommand(ShooterSubsystem shooter, HopperSubsystem hopper) {
        return Commands.either(
            Commands.sequence(
                shooter.spinUpCommand(shooter.getRPSFromTable(vision.getMeters()))
                    .until(shooter.isAtSpeedTrigger),
                hopper.runHopperCommand()
                    .until(hopper.hasPiece.negate()),
                Commands.parallel(
                    shooter.stopFlywheelCommand(),
                    hopper.stopHopperCommand()
                )
            ),
            Commands.none(),
            hopper.hasPiece
        ).withName("Shoot");
    }


    /**
     * Creates a command that ejects game pieces backward (reverse shoot).
     * Useful for clearing jams or ejecting wrong-color game pieces.
     *
     * @param shooter The shooter subsystem
     * @param hopper The hopper subsystem
     * @return Command that reverses both subsystems
     */
    public static Command ejectCommand(ShooterSubsystem shooter, HopperSubsystem hopper) {
        return Commands.parallel(
            shooter.reverseCommand(),
            hopper.reverseHopperCommand()
        ).withName("Eject");
    }
}
