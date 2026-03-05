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

    /** How long the shoot sequence runs (spin + feed) before auto-stopping. */
    public static final double kShootDurationSeconds = 5.0;

    /** Max time to wait for flywheel to reach target speed before feeding anyway. */
    public static final double kSpinUpTimeoutSeconds = 3.0;

    private ShootCommands() {}

    /**
     * Spin up → once at speed (or timeout), keep spinning + feed hopper in parallel → stop all.
     */
    public static Command shootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper) {
        return shootCommand(shooter, hopper, null, false);
    }

    /**
     * Spin up → once at speed (or timeout), keep spinning + feed hopper in parallel → stop all.
     * Optionally lowers the intake arm (without running rollers) at the start of the sequence.
     */
    public static Command shootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            boolean lowerIntake) {
        return shootCommand(shooter, hopper, intake, lowerIntake, null);
    }

    /**
     * Spin up → once at speed (or timeout), keep spinning + feed hopper in parallel → stop all.
     * When rpsSupplier is provided, uses dynamic RPS; otherwise uses fixed default.
     */
    public static Command shootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            boolean lowerIntake,
            DoubleSupplier rpsSupplier) {
        Command spinUp = rpsSupplier != null
            ? shooter.spinUpCommand(rpsSupplier)
            : shooter.spinUpCommand();
        Command spinUpFeed = rpsSupplier != null
            ? shooter.spinUpCommand(rpsSupplier)
            : shooter.spinUpCommand();

        Command sequence = Commands.sequence(
            spinUp
                .until(shooter.isAtSpeedTrigger())
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.parallel(
                spinUpFeed,
                hopper.feedCommand()
            ).withTimeout(kShootDurationSeconds),
            stopAllCommand(shooter, hopper)
        );
        if (lowerIntake && intake != null) {
            sequence = intake.runLowerIntakeArmCommand().andThen(sequence);
        }
        return sequence.withName("Shoot");
    }

    /**
     * Spin for vision distance → once at speed (or timeout), keep spinning + feed → stop all.
     */
    public static Command shootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier) {
        return shootAtDistanceCommand(shooter, hopper, distanceSupplier, null, false);
    }

    /**
     * Spin for vision distance → once at speed (or timeout), keep spinning + feed → stop all.
     * Optionally lowers the intake arm (without running rollers) at the start of the sequence.
     */
    public static Command shootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier,
            IntakeSubsystem intake,
            boolean lowerIntake) {
        Command sequence = Commands.sequence(
            shooter.spinForDistanceCommand(distanceSupplier)
                .until(shooter.isAtSpeedTrigger())
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.parallel(
                shooter.spinForDistanceCommand(distanceSupplier),
                hopper.feedCommand()
            ).withTimeout(kShootDurationSeconds),
            stopAllCommand(shooter, hopper)
        );
        if (lowerIntake && intake != null) {
            sequence = intake.runLowerIntakeArmCommand().andThen(sequence);
        }
        return sequence.withName("Shoot At Distance");
    }

    /**
     * Full auto: track turret + spin for distance + feed when ready.
     */
    public static Command autoShootCommand(
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            DoubleSupplier visionDist) {
        return Commands.sequence(
            // Track and spin up simultaneously
            Commands.parallel(
                turret.trackHubCommand(cameraAngle, cameraHasTarget),
                shooter.spinForDistanceCommand(visionDist)
            ).until(shooter.isAtSpeedTrigger().and(turret.isAtTargetTrigger()))
             .withTimeout(kSpinUpTimeoutSeconds),
            // Keep spinning + tracking while feeding
            Commands.parallel(
                turret.trackHubCommand(cameraAngle, cameraHasTarget),
                shooter.spinForDistanceCommand(visionDist),
                hopper.feedCommand()
            ).withTimeout(kShootDurationSeconds),
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
            shooter.spinUpCommand(shooter::lobShotRPS),
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
