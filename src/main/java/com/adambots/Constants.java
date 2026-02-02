// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ==================== DriveConstants ====================
    /**
     * Constants for the swerve drive system including speed limits, deadzones, and dimensions.
     */
    public static final class DriveConstants {
        /** Maximum translational speed of the robot */
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.0);
        /** Maximum rotational speed of the robot */
        public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(540);
        /** Joystick deadzone for translational movement */
        public static final double kDeadzone = 0.05;
        /** Joystick deadzone for rotational movement */
        public static final double kRotationDeadzone = 0.1;
        /** Distance between left and right wheels - TODO: Update with actual measurements */
        public static final Distance kTrackWidth = Inches.of(24);
        /** Distance between front and back wheels - TODO: Update with actual measurements */
        public static final Distance kWheelBase = Inches.of(24);
    }

    // ==================== ModuleConstants ====================
    /**
     * Constants for individual swerve modules.
     * MK5n with Kraken X60 (drive) and X44 (turn) motors.
     */
    public static final class ModuleConstants {
        /** Drive motor gear ratio for MK5n */
        public static final double kDriveGearRatio = 1.0 / 5.9;
        /** Turn motor gear ratio */
        public static final double kTurnGearRatio = 287.0 / 11.0;  // ~26.09:1
        /** Wheel diameter */
        public static final Distance kWheelDiameter = Inches.of(4);
        /** Drive motor current limit */
        public static final Current kDriveCurrentLimit = Amps.of(40);
        /** Turn motor current limit */
        public static final Current kTurnCurrentLimit = Amps.of(20);
    }

    // ==================== AutoConstants ====================
    /**
     * Constants for autonomous mode and PathPlanner.
     */
    public static final class AutoConstants {
        /** Maximum autonomous velocity */
        public static final LinearVelocity kMaxAutoSpeed = MetersPerSecond.of(4.0);
        /** Maximum autonomous acceleration */
        public static final double kMaxAutoAcceleration = 3.0;  // m/s²

        // Translation PID
        public static final double kPTranslation = 5.0;
        public static final double kITranslation = 0.0;
        public static final double kDTranslation = 0.0;

        // Rotation PID
        public static final double kPRotation = 5.0;
        public static final double kIRotation = 0.0;
        public static final double kDRotation = 0.0;
    }

    // ==================== ShooterConstants ====================
    /**
     * Constants for the shooter subsystem including flywheel speeds, turret limits, and tracking.
     */
    public static final class ShooterConstants {
        // TODO: Flywheel velocity settings
        public static final double kDefaultVelocity = 50.0;  // RPS
        // public static final double kIdleSpeed = 0.1;         // Motor power for idle
        // public static final double kVelocityTolerance = 2.0; // RPS tolerance for "at speed"

        // TODO: Turret settings
        // public static final double kTurretGearRatio = 100.0; // Motor rotations per turret rotation
        // public static final double kTurretManualSpeed = 0.5; // Max manual control speed
        // public static final double kTurretAngleTolerance = 2.0; // Degrees tolerance for "on target"

        // ==================== Hub Tracking / Scan Settings ====================
        /** Turret motor power during scan rotation (0 to 1) */
        // public static final double kScanSpeed = 0.3;

        /** Degrees the turret must rotate for a full scan */
        // public static final double kFullRotationDegrees = 360.0;

        /** Number of periodic cycles target must be lost before transitioning to SCANNING.
         *  At 50Hz (20ms periodic), 5 cycles = 100ms of target loss before scanning. */
        // public static final int kTargetLostCycles = 5;
    }

    // ==================== [MechanismName]Constants ====================
    // Add new nested classes for each mechanism
    // Example:
    // public static final class IntakeConstants {
    //     public static final double kIntakeSpeed = 0.8;
    //     public static final double kOuttakeSpeed = -0.5;
    // }
}
