package com.adambots.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for turret tracking with vision suppliers.
 */
public final class TurretCommands {

    private TurretCommands() {}

    /**
     * Continuously aims turret at the vision-supplied angle.
     * When target is lost, holds the last known position.
     */
    public static Command trackHubCommand(
            TurretSubsystem turret,
            DoubleSupplier angleSupplier,
            BooleanSupplier hasTargetSupplier) {
        return Commands.run(() -> {
            if (hasTargetSupplier.getAsBoolean()) {
                turret.setTurretAngle(angleSupplier.getAsDouble());
            }
            // When target lost, turret holds last setpoint (PID holds position)
        }, turret).withName("Track Hub");
    }

    /**
     * Slowly sweeps turret through its range searching for the hub.
     * Reverses direction at soft limits (±90°).
     */
    public static Command scanForHubCommand(TurretSubsystem turret) {
        return turret.scanCommand(TurretTrackingConstants.kScanSpeed)
            .withName("Scan For Hub");
    }

    /**
     * Auto-track: tracks when hub is visible, scans when lost.
     * Designed as a default command — runs continuously, gets interrupted
     * by explicit turret commands, resumes when they end.
     */
    public static Command autoTrackCommand(
            TurretSubsystem turret,
            DoubleSupplier angleSupplier,
            BooleanSupplier hasTargetSupplier) {
        return Commands.either(
            // When target visible: track it
            trackHubCommand(turret, angleSupplier, hasTargetSupplier),
            // When target lost: scan for it
            scanForHubCommand(turret),
            hasTargetSupplier
        ).repeatedly().withName("Auto Track Hub");
    }
}
