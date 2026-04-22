// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.adambots.Constants.VisionConstants;
import com.adambots.lib.utils.Utils;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Static field-geometry helpers — alliance-aware coordinates for game features
 * (hub center, scoring zones, etc.). Not vision-specific.
 *
 * <p>Previously these lived on {@code VisionSubsystem} which forced consumers
 * like {@link com.adambots.commands.DriveCommands#backToHubCommand} and
 * {@code TurretSubsystem.poseTrackCommand} to take a full vision dependency
 * just to read a field constant. Extracted to keep the dependency graph clean
 * and give future field helpers a home.
 *
 * <p>Coordinates come from {@link VisionConstants} — hardcoded from the
 * official field layout to eliminate field-to-field tag placement variation.
 */
public final class FieldGeometry {

    private FieldGeometry() {
        throw new UnsupportedOperationException("Utility class");
    }

    /**
     * Hub center for the robot's current alliance.
     * Convenience wrapper around {@link #getHubCenter(boolean)} using
     * {@link Utils#isOnRedAlliance()}.
     */
    public static Translation2d getHubCenter() {
        return getHubCenter(Utils.isOnRedAlliance());
    }

    /** Hub center for the specified alliance. */
    public static Translation2d getHubCenter(boolean isRed) {
        return isRed
            ? new Translation2d(VisionConstants.kRedHubCenterX, VisionConstants.kRedHubCenterY)
            : new Translation2d(VisionConstants.kBlueHubCenterX, VisionConstants.kBlueHubCenterY);
    }
}
