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
        return Commands.sequence(
            // Spin up shooter and wait until at speed
            shooter.spinUpCommand().until(shooter.isAtSpeedTrigger()),
            // Feed game piece from hopper
            hopper.feedCommand().withTimeout(0.5),
            // Stop both
            Commands.parallel(
                shooter.stopAllCommand(),
                hopper.stopCommand()
            )
        ).withName("Shoot");
    }

    /**
     * Creates a command that spins up the shooter and holds it at speed.
     * Use this to pre-spin before shooting.
     *
     * @param shooter The shooter subsystem
     * @return Command that spins up and maintains speed (runs until interrupted)
     */
    public static Command spinUpCommand(ShooterSubsystem shooter) {
        return shooter.spinUpCommand().withName("SpinUp");
    }

    /**
     * Creates a command that shoots while the shooter is already spinning.
     * Assumes shooter is at speed - just feeds from hopper.
     *
     * @param shooter The shooter subsystem (for requirements)
     * @param hopper The hopper subsystem
     * @return Command that feeds one game piece
     */
    public static Command feedAndShootCommand(ShooterSubsystem shooter, HopperSubsystem hopper) {
        return Commands.sequence(
            // Wait for shooter to be at speed (should already be)
            Commands.waitUntil(shooter.isAtSpeedTrigger()).withTimeout(0.5),
            // Feed game piece
            hopper.feedCommand().withTimeout(0.5),
            hopper.stopCommand()
        ).withName("FeedAndShoot");
    }

    /**
     * Creates a command that continuously shoots all game pieces.
     * Keeps shooter spinning and feeds until hopper is empty.
     *
     * @param shooter The shooter subsystem
     * @param hopper The hopper subsystem
     * @return Command that shoots until empty
     */
    public static Command shootAllCommand(ShooterSubsystem shooter, HopperSubsystem hopper) {
        return Commands.sequence(
            // Spin up
            shooter.spinUpCommand().until(shooter.isAtSpeedTrigger()),
            // Continuous feed until empty
            Commands.parallel(
                shooter.spinUpCommand(),
                hopper.continuousFeedCommand()
            ).until(hopper.isEmptyTrigger()),
            // Stop everything
            Commands.parallel(
                shooter.stopAllCommand(),
                hopper.stopCommand()
            )
        ).withName("ShootAll");
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
        // TODO: Implement reverse shooting
        // return Commands.parallel(
        //     shooter.reverseCommand(),
        //     hopper.reverseUptakeCommand()
        // ).withName("Eject");
        return Commands.none().withName("Eject");
    }
}
