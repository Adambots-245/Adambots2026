// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command factory for turret tracking that coordinates ShooterSubsystem with vision.
 *
 * <p>This class provides static factory methods that create commands for
 * vision-integrated turret tracking. It follows the AdambotsLib pattern where
 * subsystem-specific commands live in the subsystem, while cross-subsystem
 * coordination lives in command factories.
 *
 * <p>The vision suppliers allow this class to be decoupled from any specific
 * vision implementation (PhotonVision, Limelight, etc.).
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * // Create suppliers from your vision subsystem
 * DoubleSupplier targetAngle = () -> m_vision.getTargetYaw();
 * BooleanSupplier hasTarget = () -> m_vision.hasTarget();
 *
 * // Bind tracking commands
 * Buttons.XboxLeftBumper.whileTrue(
 *     TurretTrackingCommands.trackTargetCommand(m_shooter, targetAngle, hasTarget)
 * );
 * </pre>
 */
public final class TurretCommands {

    // Prevent instantiation
    private TurretCommands() {}

    /**
     * Creates a command that continuously tracks a target using vision-supplied angle.
     * The turret aims at the target while it's visible and holds position when lost.
     *
     * <p>This is the simplest tracking command - just aims at whatever angle is supplied.
     * For more sophisticated behavior (scan-to-reacquire), use {@link #autoTrackCommand}.
     *
     * @param shooter The shooter subsystem
     * @param targetAngle Supplier for target angle in degrees (from vision subsystem)
     * @param hasTarget Supplier indicating if target is currently visible
     * @return Command that tracks the target (runs until interrupted)
     */
    public static Command trackTargetCommand(
            ShooterSubsystem shooter,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return Commands.run(() -> {
            // TODO: If target visible, aim at it; otherwise hold position
            // if (hasTarget.getAsBoolean()) {
            //     // Target visible - aim at it
            //     // The aimTurretCommand handles this internally
            // }
            // For now, just continuously aim at the supplied angle
        }, shooter)
            .beforeStarting(() -> {
                // Could transition tracking state machine to TRACKING here
            })
            .withName("TrackTarget");
    }

    /**
     * Creates a command that tracks with scan-to-reacquire behavior.
     * When target is lost, rotates the turret to search for it.
     *
     * <p>Behavior:
     * <ul>
     *   <li>Target visible: Aim at target</li>
     *   <li>Target lost: Start scanning (rotating turret)</li>
     *   <li>Target reacquired during scan: Resume tracking</li>
     * </ul>
     *
     * <p>This command uses the ShooterSubsystem's StateMachine for state management.
     *
     * @param shooter The shooter subsystem
     * @param targetAngle Supplier for target angle in degrees
     * @param hasTarget Supplier indicating if target is visible
     * @return Command that tracks with auto-scan behavior
     */
    public static Command autoTrackCommand(
            ShooterSubsystem shooter,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        // TODO: Implement full state machine logic:
        // - TRACKING: aim at target, count lost frames
        // - SCANNING: rotate turret, check for reacquisition
        // - FAILED: hold position after full scan with no target
        //
        // The StateMachine in ShooterSubsystem can be used to manage these transitions.
        // Access via shooter's public getters if needed, or manage state locally.
        return Commands.run(() -> {
            if (hasTarget.getAsBoolean()) {
                // Target visible - aim at it
                // shooter's aimTurretCommand or internal aim method
            } else {
                // Target lost - could initiate scan
                // Consider using shooter.scanTurretCommand() here
            }
        }, shooter).withName("AutoTrack");
    }

    /**
     * Creates a command that enables tracking until the turret is on target.
     * Useful for one-shot aim-and-lock behavior before shooting.
     *
     * @param shooter The shooter subsystem
     * @param targetAngle Supplier for target angle in degrees
     * @param hasTarget Supplier indicating if target is visible
     * @return Command that tracks until on target, then stops
     */
    public static Command trackUntilOnTargetCommand(
            ShooterSubsystem shooter,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return trackTargetCommand(shooter, targetAngle, hasTarget)
            .until(shooter.isTurretAtTargetTrigger())
            .andThen(shooter.stopTrackingCommand())
            .withName("TrackUntilOnTarget");
    }

    /**
     * Creates a command that tracks while also spinning up the flywheel.
     * Useful for preparing to shoot while tracking.
     *
     * @param shooter The shooter subsystem
     * @param targetAngle Supplier for target angle in degrees
     * @param hasTarget Supplier indicating if target is visible
     * @return Command that tracks and spins up simultaneously
     */
    public static Command trackAndSpinUpCommand(
            ShooterSubsystem shooter,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return Commands.parallel(
            trackTargetCommand(shooter, targetAngle, hasTarget),
            shooter.spinUpCommand()
        ).withName("TrackAndSpinUp");
    }

    /**
     * Creates a command that tracks and waits until both on target and at speed.
     * The command completes when ready to shoot.
     *
     * @param shooter The shooter subsystem
     * @param targetAngle Supplier for target angle in degrees
     * @param hasTarget Supplier indicating if target is visible
     * @return Command that completes when ready to fire
     */
    public static Command trackUntilReadyCommand(
            ShooterSubsystem shooter,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return trackAndSpinUpCommand(shooter, targetAngle, hasTarget)
            .until(shooter.isReadyToFireTrigger())
            .withName("TrackUntilReady");
    }
}
