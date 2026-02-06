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

        // Flywheel Velocity PID Gains
        public static final double kFlywheelKP = 0.1;
        public static final double kFlywheelKI = 0.0;
        public static final double kFlywheelKD = 0.0;

        // Flywheel Feed-Forward
        // Feed-forward predicts the voltage needed to maintain a target velocity, so PID only
        // corrects for small errors. Without FF, PID has to "discover" the right voltage from
        // scratch, causing slow spin-up and oscillation. With FF, the motor reaches target speed
        // quickly and PID just fine-tunes it.
        //
        // Formula: kV = 12V / motorFreeSpeedRPS
        // At free speed, the motor needs full voltage (12V), so this ratio gives volts-per-RPS.
        //
        // Common motor free speeds (from datasheets):
        //   Kraken X60: 6000 RPM = 100 RPS
        //   Kraken X44: 7530 RPM = 125.5 RPS
        //   Minion:     7700 RPM = 128.3 RPS
        //
        // TODO: Set to your flywheel motor's free speed in RPS
        public static final double kFlywheelMaxFreeSpeedRPS = 100.0;  // Kraken X60 free speed
        public static final double kFlywheelFF = 12.0 / kFlywheelMaxFreeSpeedRPS;  // Volts per RPS

        public static final double kFlywheelTolerance = 2.0;  // RPS tolerance for "at speed"

        // Turret settings
        public static final double kTurretGearRatio = 100.0; // Motor rotations per turret rotation
        public static final double kTurretManualSpeed = 0.5; // Max manual control speed
        public static final double kTurretAngleTolerance = 2.0; // Degrees tolerance for "on target"

        // Turret Position PID Gains
        public static final double kTurretKP = 0.05;
        public static final double kTurretKI = 0.0;
        public static final double kTurretKD = 0.0;

        // TODO: Turret encoder DIO port (REV Through Bore Encoder)
        // public static final int kTurretEncoderPort = 0;  // DIO port 0-9

        // TODO: Turret encoder offset (degrees) - align encoder zero with turret forward
        // public static final double kTurretEncoderOffset = 0.0;

        // ==================== Hub Tracking / Scan Settings ====================
        /** Turret motor power during scan rotation (0 to 1) */
        // public static final double kScanSpeed = 0.3;

        /** Degrees the turret must rotate for a full scan */
        // public static final double kFullRotationDegrees = 360.0;

        /** Number of periodic cycles target must be lost before transitioning to SCANNING.
         *  At 50Hz (20ms periodic), 5 cycles = 100ms of target loss before scanning. */
        // public static final int kTargetLostCycles = 5;
    }

    // ==================== VisionConstants ====================
    /**
     * Constants for the vision system including camera names, positions, and pose estimation parameters.
     *
     * <p>Camera Position Coordinate System (robot-centric):
     * <ul>
     *   <li>X: Forward is positive (towards front of robot)</li>
     *   <li>Y: Left is positive (towards left side of robot)</li>
     *   <li>Z: Up is positive (towards top of robot)</li>
     * </ul>
     *
     * <p>Camera Rotation (Euler angles):
     * <ul>
     *   <li>Roll: Rotation around X-axis (camera tilting side-to-side)</li>
     *   <li>Pitch: Rotation around Y-axis (camera tilting up/down, negative = pitched down)</li>
     *   <li>Yaw: Rotation around Z-axis (camera rotated left/right, 0 = facing forward)</li>
     * </ul>
     */
    public static final class VisionConstants {
        // ==================== Camera Names ====================
        // These must match the camera names configured in PhotonVision

        /** Left odometry camera name (Arducam OV9281 on front-left swerve module) */
        public static final String kLeftOdomCameraName = "left_odom";

        /** Right odometry camera name (Arducam OV9281 on front-right swerve module) */
        public static final String kRightOdomCameraName = "right_odom";

        /** Turret camera name (Microsoft LifeCam HD-3000 on turret) */
        public static final String kTurretCameraName = "turret";

        /** Human player station camera name */
        public static final String kHPStationCameraName = "hp_station";

        // ==================== Camera Positions ====================
        // TODO: Update these values with actual measurements from CAD
        // All measurements are from robot center (between swerve modules at floor level)

        // Left Odometry Camera (front-left swerve module)
        /** X position of left odom camera in meters (forward from robot center) */
        public static final double kLeftOdomCameraX = 0.3;  // TODO: Measure from CAD
        /** Y position of left odom camera in meters (left from robot center) */
        public static final double kLeftOdomCameraY = 0.3;  // TODO: Measure from CAD
        /** Z position of left odom camera in meters (up from floor) */
        public static final double kLeftOdomCameraZ = 0.2;  // TODO: Measure from CAD
        /** Roll of left odom camera in degrees */
        public static final double kLeftOdomCameraRoll = 0.0;
        /** Pitch of left odom camera in degrees (negative = pitched down) */
        public static final double kLeftOdomCameraPitch = -15.0;
        /** Yaw of left odom camera in degrees */
        public static final double kLeftOdomCameraYaw = 0.0;

        // Right Odometry Camera (front-right swerve module)
        /** X position of right odom camera in meters (forward from robot center) */
        public static final double kRightOdomCameraX = 0.3;  // TODO: Measure from CAD
        /** Y position of right odom camera in meters (right from robot center, negative) */
        public static final double kRightOdomCameraY = -0.3;  // TODO: Measure from CAD
        /** Z position of right odom camera in meters (up from floor) */
        public static final double kRightOdomCameraZ = 0.2;  // TODO: Measure from CAD
        /** Roll of right odom camera in degrees */
        public static final double kRightOdomCameraRoll = 0.0;
        /** Pitch of right odom camera in degrees (negative = pitched down) */
        public static final double kRightOdomCameraPitch = -15.0;
        /** Yaw of right odom camera in degrees */
        public static final double kRightOdomCameraYaw = 0.0;

        // Turret Camera (mounted on turret, centered on rotation axis)
        /** X position of turret camera in meters (forward from turret center) */
        public static final double kTurretCameraX = 0.1;  // TODO: Measure from CAD
        /** Y position of turret camera in meters (should be ~0 if centered) */
        public static final double kTurretCameraY = 0.0;
        /** Z position of turret camera in meters (up from turret base) */
        public static final double kTurretCameraZ = 0.15;  // TODO: Measure from CAD
        /** Roll of turret camera in degrees */
        public static final double kTurretCameraRoll = 0.0;
        /** Pitch of turret camera in degrees */
        public static final double kTurretCameraPitch = 0.0;
        /** Yaw of turret camera in degrees (relative to turret forward) */
        public static final double kTurretCameraYaw = 0.0;

        // ==================== Pose Estimation Parameters ====================

        /**
         * Ambiguity threshold for rejecting poses (0.0 to 1.0).
         * Lower = more strict, rejects more ambiguous poses.
         * Recommended: 0.2 for competition, 0.3 for testing.
         */
        public static final double kAmbiguityThreshold = 0.2;

        /**
         * Standard deviations for single-tag pose estimation.
         * Higher values = less trust in the measurement.
         * Format: {x meters, y meters, rotation radians}
         */
        public static final double[] kSingleTagStdDevs = {0.5, 0.5, 1.0};

        /**
         * Standard deviations for multi-tag pose estimation.
         * Multi-tag is more accurate, so lower std devs.
         * Format: {x meters, y meters, rotation radians}
         */
        public static final double[] kMultiTagStdDevs = {0.2, 0.2, 0.5};

        // ==================== AprilTag Groups ====================
        // Game-specific tag groups for filtering and detection
        // Use with vision.hasID() or allowedTags() in VisionConfigBuilder
        // Reference: plans/vision-implementation.md for full tag layout

        // TODO: Verify tag IDs match 2026 REBUILT field layout

        /** Red alliance HUB tags - primary targets for scoring */
        public static final int[] kRedHubTags = {};  // TODO: Add tag IDs

        /** Blue alliance HUB tags - primary targets for scoring */
        public static final int[] kBlueHubTags = {};  // TODO: Add tag IDs

        /** Red alliance TOWER WALL tags - good for pose estimation */
        public static final int[] kRedTowerTags = {};  // TODO: Add tag IDs

        /** Blue alliance TOWER WALL tags - good for pose estimation */
        public static final int[] kBlueTowerTags = {};  // TODO: Add tag IDs

        /** Red alliance TRENCH tags */
        public static final int[] kRedTrenchTags = {};  // TODO: Add tag IDs

        /** Blue alliance TRENCH tags */
        public static final int[] kBlueTrenchTags = {};  // TODO: Add tag IDs

        /**
         * Gets the HUB tags for the current alliance.
         * @param alliance The current alliance from DriverStation
         * @return Array of HUB tag IDs for the alliance
         */
        public static int[] getHubTags(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? kRedHubTags : kBlueHubTags;
        }

        /**
         * Gets the HUB tags for the current alliance.
         * Use with Utils.isOnRedAlliance() for cleaner code.
         * @param isRedAlliance true if on red alliance, false if on blue
         * @return Array of HUB tag IDs for the alliance
         */
        public static int[] getHubTags(boolean isRedAlliance) {
            return isRedAlliance ? kRedHubTags : kBlueHubTags;
        }

        /**
         * Gets the TOWER WALL tags for the current alliance.
         * @param alliance The current alliance from DriverStation
         * @return Array of TOWER tag IDs for the alliance
         */
        public static int[] getTowerTags(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? kRedTowerTags : kBlueTowerTags;
        }

        /**
         * Gets the TOWER WALL tags for the current alliance.
         * Use with Utils.isOnRedAlliance() for cleaner code.
         * @param isRedAlliance true if on red alliance, false if on blue
         * @return Array of TOWER tag IDs for the alliance
         */
        public static int[] getTowerTags(boolean isRedAlliance) {
            return isRedAlliance ? kRedTowerTags : kBlueTowerTags;
        }

        // ==================== Network Configuration ====================

        /** PhotonVision coprocessor IP address (OrangePi) */
        public static final String kPhotonVisionIP = "10.2.45.11";

        /** PhotonVision web dashboard port */
        public static final int kPhotonVisionPort = 5800;
    }

    // ==================== [MechanismName]Constants ====================
    // Add new nested classes for each mechanism
    // Example:
    // public static final class IntakeConstants {
    //     public static final double kIntakeSpeed = 0.8;
    //     public static final double kOuttakeSpeed = -0.5;
    // }
}
