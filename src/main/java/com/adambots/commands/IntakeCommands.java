// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for coordinating intake and hopper subsystems.
 *
 * <p>This class provides static factory methods that create commands
 * requiring coordination between the intake and hopper.
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * Buttons.XboxAButton.whileTrue(IntakeCommands.intakeAndIndexCommand(m_intake, m_hopper));
 * </pre>
 */
public final class IntakeCommands {

    // Prevent instantiation
    private IntakeCommands() {}

    /**
     * Creates a command that intakes a game piece and indexes it into the hopper.
     * Runs intake until game piece detected, then runs hopper to index.
     *
     * @param intake The intake subsystem
     * @param hopper The hopper subsystem
     * @return Command that intakes and indexes one game piece
     */
    public static Command intakeAndIndexCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        return Commands.sequence(
            // Run intake until game piece detected
            intake.intakeCommand().until(intake.hasGamePieceTrigger()),
            // Stop intake
            intake.stopCommand(),
            // Index into hopper
            hopper.feedCommand()
        ).withName("IntakeAndIndex");
    }

    /**
     * Creates a command that clears a jam by reversing briefly then resuming intake.
     *
     * @param intake The intake subsystem
     * @param hopper The hopper subsystem
     * @return Command that clears jam and resumes
     */
    public static Command clearJamCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        return Commands.sequence(
            // Reverse briefly
            Commands.parallel(
                intake.outtakeCommand(),
                hopper.reverseCarouselCommand()
            ).withTimeout(0.3),
            // Stop
            Commands.parallel(
                intake.stopCommand(),
                hopper.stopCommand()
            )
        ).withName("ClearJam");
    }
}
