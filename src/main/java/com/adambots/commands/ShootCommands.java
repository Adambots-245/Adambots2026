package com.adambots.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.adambots.Constants;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.VisionSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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

    /** Fire gate debounce — speed must hold steady for this long before feeding.
     *  With 2.5 RPS tolerance, 0.08s is sufficient to filter oscillation without
     *  adding excessive delay to the spin-up pipeline (~700ms + debounce). */
    public static final double kFireGateDebounceSeconds = 0.08;

    private ShootCommands() {}

    /**
     * Creates a gated feed command: hopper only feeds while the flywheel is at speed.
     * When the flywheel drops below tolerance (e.g. during a shot RPM dip that exceeds
     * the tolerance band), the hopper pauses until speed recovers. This prevents balls
     * from being launched at under-speed and producing inconsistent shots.
     */
    private static Command gatedFeedCommand(HopperSubsystem hopper, ShooterSubsystem shooter) {
        return hopper.gatedFeedCommand(shooter::isAtSpeed);
    }

    /**
     * Wraps a shoot command to suppress turret auto-tracking while shooting.
     * Prevents the turret from sweeping if vision drops momentarily during a shot.
     * Restores the previous auto-track state when the command ends.
     */
    private static Command withAutoTrackSuppressed(Command shootCommand, TurretSubsystem turret) {
        return withAutoTrackSuppressed(shootCommand, turret, null);
    }

    private static Command withAutoTrackSuppressed(Command shootCommand, TurretSubsystem turret, VisionSubsystem vision) {
        final boolean[] wasAutoTracking = {false};
        return Commands.runOnce(() -> {
            wasAutoTracking[0] = turret.isAutoTrackEnabled();
            turret.setAutoTrack(false);
            if (vision != null) vision.setShootingMode(true);
        }).andThen(shootCommand)
          .finallyDo(() -> {
              if (wasAutoTracking[0]) turret.setAutoTrack(true);
              if (vision != null) vision.setShootingMode(false);
          });
    }

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
                .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.parallel(
                spinUpFeed,
                gatedFeedCommand(hopper, shooter)
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
    public static Command shootAtDistanceTimerCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier) {
        return shootAtDistanceTimerCommand(shooter, hopper, distanceSupplier, null, false);
    }

    /**
     * Spin for vision distance with turret auto-track suppression → stop all.
     */
    public static Command shootAtDistanceTimerCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            TurretSubsystem turret,
            DoubleSupplier distanceSupplier) {
        return shootAtDistanceTimerCommand(shooter, hopper, turret, distanceSupplier, null);
    }

    public static Command shootAtDistanceTimerCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            TurretSubsystem turret,
            DoubleSupplier distanceSupplier,
            VisionSubsystem vision) {
        return withAutoTrackSuppressed(
            shootAtDistanceTimerCommand(shooter, hopper, distanceSupplier), turret, vision)
            .withName("Shoot At Distance");
    }

    /**
     * Spin for vision distance → once at speed (or timeout), keep spinning + feed → stop all.
     * Optionally lowers the intake arm (without running rollers) at the start of the sequence.
     */
    public static Command shootAtDistanceTimerCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier,
            IntakeSubsystem intake,
            boolean lowerIntake) {
        Command sequence = Commands.sequence(
            shooter.spinForDistanceCommand(distanceSupplier)
                .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.defer(() -> {
                double dist = distanceSupplier.getAsDouble();
                shooter.setShotBoost(true);
                return Commands.parallel(
                    shooter.spinForDistanceCommand(() -> dist),
                    gatedFeedCommand(hopper, shooter)
                ).withTimeout(kShootDurationSeconds);
            }, Set.of(shooter, hopper)),
            stopAllCommand(shooter, hopper)
        ).finallyDo(() -> shooter.setShotBoost(false));
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
            DoubleSupplier visionDist) {
        return Commands.sequence(
            // Track and spin up simultaneously (turret uses default command for tracking)
            shooter.spinForDistanceCommand(visionDist)
                .until(shooter.isAtSpeedTrigger().and(turret.isAtTargetTrigger()).debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            // Snapshot distance and feed with locked RPS + shot boost
            Commands.defer(() -> {
                double dist = visionDist.getAsDouble();
                shooter.setShotBoost(true);
                return Commands.parallel(
                    shooter.spinForDistanceCommand(() -> dist),
                    gatedFeedCommand(hopper, shooter)
                ).withTimeout(kShootDurationSeconds);
            }, Set.of(shooter, hopper)),
            stopAllCommand(shooter, hopper)
        ).finallyDo(() -> shooter.setShotBoost(false))
         .withName("Auto Shoot");
    }

    // ==================== HOLD-TO-SHOOT (no timer) ====================

    /**
     * Hold-to-shoot: spin up → wait for speed → spin + feed until released.
     * No timeouts — runs entirely while the trigger is held.
     */
    public static Command holdShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper) {
        return holdShootCommand(shooter, hopper, (DoubleSupplier) null);
    }

    /**
     * Hold-to-shoot with dynamic RPS: spin up → wait for speed → spin + feed until released.
     * No timeouts — runs entirely while the trigger is held.
     */
    public static Command holdShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier rpsSupplier) {
        Command spinUp = rpsSupplier != null
            ? shooter.spinUpCommand(rpsSupplier)
            : shooter.spinUpCommand();
        Command spinUpFeed = rpsSupplier != null
            ? shooter.spinUpCommand(rpsSupplier)
            : shooter.spinUpCommand();

        return Commands.sequence(
            spinUp.until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds)),
            Commands.parallel(spinUpFeed, gatedFeedCommand(hopper, shooter))
        ).finallyDo(() -> {
            shooter.stopFlywheel();
        }).withName("Hold Shoot");
    }

    public static Command holdShootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            TurretSubsystem turret,
            SwerveSubsystem swerve,
            DoubleSupplier distanceSupplier,
            VisionSubsystem vision) {
        return Commands.parallel(
            withAutoTrackSuppressed(
                holdShootAtDistanceCommand(shooter, hopper, distanceSupplier), turret, vision),
            shakeCommand(swerve))
            .withName("Hold Shoot At Distance");
    }

    /**
     * Hold-to-shoot at vision distance: spin for distance → wait for speed → spin + feed until released.
     * No timeouts — runs entirely while the trigger is held.
     */
    public static Command holdShootAtDistanceCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier distanceSupplier) {
        return Commands.sequence(
            shooter.spinForDistanceCommand(distanceSupplier)
                .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.defer(() -> {
                double dist = distanceSupplier.getAsDouble();
                shooter.setShotBoost(true);
                return Commands.parallel(
                    shooter.spinForDistanceCommand(() -> dist),
                    gatedFeedCommand(hopper, shooter));
            }, Set.of(shooter, hopper))
        ).finallyDo(() -> {
            shooter.setShotBoost(false);
            shooter.stopFlywheel();
        }).withName("Hold Shoot At Distance");
    }

    /**
     * Hold-to-shoot at vision distance while bopping the intake arm to shake balls
     * toward the hopper. No timeouts — runs entirely while the trigger is held.
     */
    public static Command holdShootAtDistanceWithBopCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            DoubleSupplier distanceSupplier) {
        return Commands.sequence(
            Commands.deadline(
                shooter.spinForDistanceCommand(distanceSupplier)
                    .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                    .withTimeout(kSpinUpTimeoutSeconds),
                intake.bopArmCommand()),
            Commands.defer(() -> {
                double dist = distanceSupplier.getAsDouble();
                shooter.setShotBoost(true);
                return Commands.parallel(
                    shooter.spinForDistanceCommand(() -> dist),
                    gatedFeedCommand(hopper, shooter),
                    intake.bopArmCommand());
            }, Set.of(shooter, hopper, intake))
        ).finallyDo(() -> {
            shooter.setShotBoost(false);
            shooter.stopFlywheel();
        }).withName("Hold Shoot At Distance With Bop");
    }

    /**
     * Timer-based shoot at distance with bop and turret auto-track suppression.
     */
    public static Command shootAtDistanceTimerWithBopCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            DoubleSupplier distanceSupplier) {
        return shootAtDistanceTimerWithBopCommand(shooter, hopper, intake, turret, distanceSupplier, null);
    }

    public static Command shootAtDistanceTimerWithBopCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            DoubleSupplier distanceSupplier,
            VisionSubsystem vision) {
        return withAutoTrackSuppressed(
            shootAtDistanceTimerWithBopCommand(shooter, hopper, intake, distanceSupplier), turret, vision)
            .withName("Shoot At Distance With Bop");
    }

    /**
     * Timer-based shoot at distance with bop: spin for distance + bop → once at speed (or timeout),
     * keep spinning + feed + bop → stop all. For autonomous / PathPlanner use.
     */
    public static Command shootAtDistanceTimerWithBopCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            DoubleSupplier distanceSupplier) {
        return Commands.sequence(
            Commands.deadline(
                shooter.spinForDistanceCommand(distanceSupplier)
                    .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                    .withTimeout(kSpinUpTimeoutSeconds),
                intake.bopArmCommand()),
            Commands.defer(() -> {
                double dist = distanceSupplier.getAsDouble();
                shooter.setShotBoost(true);
                return Commands.parallel(
                    shooter.spinForDistanceCommand(() -> dist),
                    gatedFeedCommand(hopper, shooter),
                    intake.bopArmCommand()
                ).withTimeout(kShootDurationSeconds);
            }, Set.of(shooter, hopper, intake)),
            stopAllCommand(shooter, hopper)
        ).finallyDo(() -> shooter.setShotBoost(false))
         .withName("Shoot At Distance With Bop");
    }

    /**
     * Lob shot: spin flywheel to lob RPS → wait for speed → intake + feed while held.
     * Hold button to run, release to stop all.
     */
    public static Command lobShotCommand(
            ShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake) {
        return Commands.sequence(
            shooter.spinUpCommand(shooter::lobShotRPS)
                .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.parallel(
                intake.runIntakeCommand(),
                shooter.spinUpCommand(shooter::lobShotRPS),
                gatedFeedCommand(hopper, shooter))
        ).finallyDo(() -> shooter.stopFlywheel())
         .withName("Lob Shot");
    }

    /**
     * Manual hold-to-shoot: driver controls flywheel speed via throttle, operator holds button to shoot.
     * Throttle maps to the interpolation table's RPS range (min→max). No vision dependency.
     * Spin-up uses throttle continuously; once at speed, feeds while held.
     */
    public static Command manualShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DoubleSupplier throttleSupplier) {
        DoubleSupplier rpsSupplier = () -> shooter.throttleToRPS(throttleSupplier.getAsDouble());
        return Commands.sequence(
            shooter.spinUpCommand(rpsSupplier)
                .until(shooter.isAtSpeedTrigger().debounce(kFireGateDebounceSeconds))
                .withTimeout(kSpinUpTimeoutSeconds),
            Commands.runOnce(() -> shooter.setShotBoost(true)),
            Commands.parallel(
                shooter.spinUpCommand(rpsSupplier),
                gatedFeedCommand(hopper, shooter))
        ).finallyDo(() -> {
            shooter.setShotBoost(false);
            shooter.stopFlywheel();
        }).withName("Manual Shoot (Throttle)");
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

    /**
     * AUTON ONLY: Lob fuel from the center
     */
    public static Command autonLobCommand(ShooterSubsystem shooter, TurretSubsystem turret, HopperSubsystem hopper, IntakeSubsystem intake) {
        return Commands.parallel(
            turret.aimTurretCommand(()->90),
            lobShotCommand(shooter, hopper, intake)
        ).withName("autonLob");
    }

    // ==================== Chassis Shake ====================

    /**
     * Shake command: oscillates the chassis rotationally to settle balls into the carousel.
     * Returns a no-op when {@code ShooterConstants.kShakeEnabled} is false.
     * Uses rotation instead of translation so the robot stays in place and the
     * turret's angular velocity feedforward keeps it on target.
     */
    public static Command shakeCommand(SwerveSubsystem swerve) {
        if (!Constants.ShooterConstants.kShakeEnabled) {
            return Commands.none();
        }
        Timer shakeTimer = new Timer();
        return Commands.runOnce(shakeTimer::restart)
            .andThen(swerve.driveFieldOrientedCommand(() -> {
                double elapsed = shakeTimer.get() % Constants.ShooterConstants.kShakePeriodSeconds;
                double direction = (elapsed < Constants.ShooterConstants.kShakePeriodSeconds / 2) ? 1.0 : -1.0;
                return new ChassisSpeeds(0, 0, direction * Constants.ShooterConstants.kShakeRotSpeed);
            }))
            .finallyDo(interrupted -> swerve.setChassisSpeeds(new ChassisSpeeds()))
            .withName("Chassis Shake");
    }

}
