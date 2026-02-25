package com.adambots.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.UptakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory coordinating shooter, hopper, uptake, and turret subsystems.
 */
public final class ShootCommands {

    private ShootCommands() {}

    /**
     * Spin up → wait for speed → feed hopper + uptake → stop all.
     */
    public static Command shootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            UptakeSubsystem uptake) {
        return Commands.sequence(
            shooter.spinUpCommand().until(shooter.isAtSpeedTrigger()),
            Commands.parallel(
                hopper.feedCommand(),
                uptake.runUptakeCommand()
            ).withTimeout(1.0),
            stopAllCommand(shooter, hopper, uptake)
        ).withName("Shoot");
    }

    /**
     * Spin for vision distance → wait for speed → feed hopper + uptake → stop all.
     */
    public static Command shootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            UptakeSubsystem uptake,
            DoubleSupplier distanceSupplier) {
        return Commands.sequence(
            shooter.spinForDistanceCommand(distanceSupplier).until(shooter.isAtSpeedTrigger()),
            Commands.parallel(
                hopper.feedCommand(),
                uptake.runUptakeCommand()
            ).withTimeout(1.0),
            stopAllCommand(shooter, hopper, uptake)
        ).withName("Shoot At Distance");
    }

    /**
     * Full auto: track turret + spin for distance + feed when ready.
     */
    public static Command autoShootCommand(
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            UptakeSubsystem uptake,
            DoubleSupplier visionAngle,
            DoubleSupplier visionDist,
            BooleanSupplier hasTarget) {
        return Commands.sequence(
            // Track and spin up simultaneously
            Commands.parallel(
                TurretCommands.trackHubCommand(turret, visionAngle, hasTarget),
                shooter.spinForDistanceCommand(visionDist)
            ).until(shooter.isAtSpeedTrigger().and(turret.isAtTargetTrigger())),
            // Feed when ready
            Commands.parallel(
                hopper.feedCommand(),
                uptake.runUptakeCommand()
            ).withTimeout(1.0),
            stopAllCommand(shooter, hopper, uptake)
        ).withName("Auto Shoot");
    }

    /**
     * Reverse all to clear jams.
     */
    public static Command ejectCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            UptakeSubsystem uptake) {
        return Commands.parallel(
            hopper.reverseHopperCommand(),
            uptake.reverseUptakeCommand()
        ).withName("Eject");
    }

    /**
     * Stop all shooter-related subsystems.
     */
    public static Command stopAllCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            UptakeSubsystem uptake) {
        return Commands.parallel(
            shooter.stopFlywheelCommand(),
            hopper.stopHopperCommand(),
            uptake.stopUptakeCommand()
        ).withName("Stop All");
    }
}
