package com.adambots.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory coordinating shooter, hopper, and turret subsystems.
 */
public final class ShootCommands {

    private ShootCommands() {}

    /**
     * Spin up → wait for speed → feed hopper + uptake → stop all.
     */
    public static Command shootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper) {
        return Commands.sequence(
            shooter.spinUpCommand().until(shooter.isAtSpeedTrigger()),
            hopper.feedCommand().withTimeout(5.0),
            stopAllCommand(shooter, hopper)
        ).withName("Shoot");
    }

    /**
     * Spin for vision distance → wait for speed → feed hopper + uptake → stop all.
     */
    public static Command shootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier) {
        return Commands.sequence(
            shooter.spinUpCommand().until(shooter.isAtSpeedTrigger()),
            shooter.spinForDistanceCommand(distanceSupplier).until(shooter.isAtSpeedTrigger()),
            hopper.feedCommand().withTimeout(10.0),
            stopAllCommand(shooter, hopper)
        ).withName("Shoot At Distance");
    }

    /**
     * Full auto: track turret + spin for distance + feed when ready.
     */
    public static Command autoShootCommand(
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            DoubleSupplier visionAngle,
            DoubleSupplier visionDist,
            BooleanSupplier hasTarget) {
        return Commands.sequence(
            // Track and spin up simultaneously
            Commands.parallel(
                turret.trackHubCommand(visionAngle, hasTarget),
                shooter.spinForDistanceCommand(visionDist)
            ).until(shooter.isAtSpeedTrigger().and(turret.isAtTargetTrigger())),
            // Feed when ready
            hopper.feedCommand().withTimeout(1.0),
            stopAllCommand(shooter, hopper)
        ).withName("Auto Shoot");
    }

    /**
     * Lob shot: simultaneously intake + spin flywheel at fixed RPS + feed hopper.
     * Hold button to run, release to stop all.
     */
    public static Command lobShotCommand(
            ShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake) {
        return Commands.parallel(
            intake.runIntakeCommand(),
            shooter.spinUpCommand(shooter.lobShotRPS()),
            hopper.feedCommand()
        ).withName("Lob Shot");
    }

    /**
     * Reverse hopper/uptake and stop flywheel to clear jams.
     */
    public static Command ejectCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper) {
        return Commands.parallel(
            hopper.reverseCommand(),
            shooter.stopFlywheelCommand()
        ).withName("Eject");
    }

    /**
     * Stop all shooter-related subsystems.
     */
    public static Command stopAllCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper) {
        return Commands.parallel(
            shooter.stopFlywheelCommand(),
            hopper.stopCommand()
        ).withName("Stop All");
    }
}
