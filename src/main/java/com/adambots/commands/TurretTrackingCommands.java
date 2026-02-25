package com.adambots.commands;

import com.adambots.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Legacy turret tracking commands — delegates to {@link TurretCommands}.
 * Kept for backward compatibility with any PathPlanner references.
 */
public final class TurretTrackingCommands {

    private TurretTrackingCommands() {}

    public static Command trackTargetCommand(
            TurretSubsystem turret,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return TurretCommands.trackHubCommand(turret, targetAngle, hasTarget);
    }

    public static Command autoTrackCommand(
            TurretSubsystem turret,
            DoubleSupplier targetAngle,
            BooleanSupplier hasTarget) {
        return TurretCommands.autoTrackCommand(turret, targetAngle, hasTarget);
    }
}
