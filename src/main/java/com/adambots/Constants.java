// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        // Flywheel velocity settings
        // Target: 3586 RPM (62% of max) = 59.8 RPS
        public static final double kDefaultVelocity = 59.8;  // RPS
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

        // ==================== Hub Centers ====================
        // Field positions of alliance hub centers (for distance calculations)
        /** Blue alliance hub center position on the field (meters) */
        public static final Translation2d kBlueHubCenter = new Translation2d(4.62, 4.03);
        /** Red alliance hub center position on the field (meters) */
        public static final Translation2d kRedHubCenter = new Translation2d(12.43, 4.03);

        // ==================== AprilTag Groups ====================
        // Game-specific tag groups for filtering and detection
        // Use with vision.hasID() or allowedTags() in VisionConfigBuilder
        // Reference: plans/vision-implementation.md for full tag layout

        // TODO: Verify tag IDs match 2026 REBUILT field layout

        /** Red alliance HUB tags - primary targets for scoring */
        public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

        /** Blue alliance HUB tags - primary targets for scoring */
        public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

        /** Red alliance TOWER WALL tags - good for pose estimation */
        public static final int[] kRedTowerTags = {13, 14, 15, 16};

        /** Blue alliance TOWER WALL tags - good for pose estimation */
        public static final int[] kBlueTowerTags = {29, 30, 31, 32};

        /** Red alliance TRENCH tags */
        public static final int[] kRedTrenchTags = {1, 6, 7, 12};

        /** Blue alliance TRENCH tags */
        public static final int[] kBlueTrenchTags = {17, 22, 23, 28};

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

        /**
         * Gets the hub center position for the current alliance.
         * @param isRedAlliance true if on red alliance, false if on blue
         * @return Translation2d of the hub center
         */
        public static Translation2d getHubCenter(boolean isRedAlliance) {
            return isRedAlliance ? kRedHubCenter : kBlueHubCenter;
        }

        // ==================== Network Configuration ====================

        /** PhotonVision coprocessor IP address (OrangePi) */
        public static final String kPhotonVisionIP = "10.2.45.11";

        /** PhotonVision web dashboard port */
        public static final int kPhotonVisionPort = 5800;
    }

    // ==================== SimulationConstants ====================
    /**
     * Constants for 3D simulation with AdvantageScope, including FUEL physics
     * and PhotonVision camera simulation parameters.
     */
    public static final class SimulationConstants {
        // === FUEL Game Piece Physics (2026) ===
        /** FUEL diameter in meters (15.0 cm / 5.91 inches) */
        public static final double kFuelDiameterMeters = 0.15;
        /** FUEL mass in kg (0.227 kg / 0.5 lbs) - confirmed by mechanical team */
        public static final double kFuelMassKg = 0.227;
        /** Drag coefficient for FUEL (sphere approximation) */
        public static final double kFuelDragCoefficient = 0.47;

        // === Shooter Physics (confirmed by mechanical team) ===
        /** Fixed hood angle in degrees (measured from horizontal) - confirmed 60° */
        public static final double kHoodAngleDegrees = 60.0;
        /** Launch height from ground in meters - TODO: waiting for exit height from mechanical */
        public static final double kLaunchHeightMeters = 0.45;
        /** Flywheel wheel radius in meters (2 inches = 0.0508m) - confirmed */
        public static final double kFlywheelRadiusMeters = 0.0508;
        /** Exit velocity multiplier (accounts for slip, typically 0.7-0.9) */
        public static final double kExitVelocityMultiplier = 0.85;

        // === Camera Simulation ===
        /** Camera resolution width in pixels */
        public static final int kCameraResolutionWidth = 1280;
        /** Camera resolution height in pixels */
        public static final int kCameraResolutionHeight = 720;
        /** Camera horizontal FOV in degrees */
        public static final double kCameraFOVDegrees = 70.0;
        /** Camera position relative to robot center (X = forward) */
        public static final double kCameraXMeters = 0.3;
        /** Camera position relative to robot center (Y = left) */
        public static final double kCameraYMeters = 0.0;
        /** Camera position relative to robot center (Z = up) */
        public static final double kCameraZMeters = 0.5;
        /** Camera pitch angle in degrees (negative = looking down) */
        public static final double kCameraPitchDegrees = -20.0;
        /** Camera yaw angle in degrees (0 = forward) */
        public static final double kCameraYawDegrees = 0.0;

        // === Target Heights (2026 Hub) ===
        /** Low hub target height in meters */
        public static final double kLowHubHeightMeters = 1.04;
        /** High hub target height in meters (72 inches = 1.83m) - from game manual */
        public static final double kHighHubHeightMeters = 1.83;
    }

    // ==================== TestTurretConstants ====================
    /**
     * Constants for the test turret subsystem used to verify PIDAutoTuner functionality.
     *
     * <p>This is a dummy turret for testing purposes. The physics constants are
     * tuned to provide realistic simulation response for PID tuning experiments.
     */
    public static final class TestTurretConstants {
        // Motor settings
        /** Gear ratio: motor rotations per turret rotation */
        public static final double kGearRatio = 100.0;  // 100:1 reduction

        // Position limits (degrees)
        /** Minimum turret angle in degrees */
        public static final double kMinAngle = -180.0;
        /** Maximum turret angle in degrees */
        public static final double kMaxAngle = 180.0;

        // Simulation physics
        /** Moment of inertia in kg*m^2 (affects acceleration response) */
        public static final double kMomentOfInertia = 0.1;
        /** Friction coefficient (affects damping and steady-state) */
        public static final double kFrictionCoefficient = 0.02;

        // Default PID (before tuning)
        /** Default proportional gain */
        public static final double kP = 0.05;
        /** Default integral gain */
        public static final double kI = 0.0;
        /** Default derivative gain */
        public static final double kD = 0.0;
    }

    // ==================== HangConstants ====================
    /**
     * Constants for simulated hang alignment using a rear-facing PhotonVision camera.
     *
     * <p>The hang mechanism mounts to the back of the robot and climbs from the side
     * of the tower structure. A rear camera detects tower wall AprilTags and three
     * PID controllers align rotation, lateral position, and standoff distance.
     *
     * <p>Tower Wall AprilTags are at z=0.55m on alliance walls:
     * <ul>
     *   <li>Red Upper: tags 13,14 at x=16.533m, y≈7.19m facing -X</li>
     *   <li>Red Lower: tags 15,16 at x=16.533m, y≈4.11m facing -X</li>
     *   <li>Blue Lower: tags 29,30 at x=0.008m, y≈0.88m facing +X</li>
     *   <li>Blue Upper: tags 31,32 at x=0.008m, y≈3.96m facing +X</li>
     * </ul>
     */
    public static final class HangConstants {
        // ==================== Rear Camera Position ====================
        /** Camera X: behind robot center (negative = backward) */
        public static final double kRearCameraXMeters = -0.28;
        /** Camera Y: centered on robot */
        public static final double kRearCameraYMeters = 0.0;
        /** Camera Z: 12 inches up from ground */
        public static final double kRearCameraZMeters = 0.30;
        /** Camera pitch: slightly up to see low-mounted tags (degrees) */
        public static final double kRearCameraPitchDegrees = 5.0;
        /** Camera yaw: facing backward (degrees) */
        public static final double kRearCameraYawDegrees = 180.0;
        /** Camera name in PhotonVision */
        public static final String kRearCameraName = "hang_camera";

        // ==================== Target Distance ====================
        /** Target distance from robot center to wall (meters) */
        public static final double kTargetDistanceFromWall = 0.60;

        // ==================== PID Gains ====================
        // Rotation PID (heading alignment)
        public static final double kRotP = 3.0;
        public static final double kRotI = 0.0;
        public static final double kRotD = 0.1;
        public static final double kRotToleranceDeg = 2.0;
        public static final double kMaxOmega = 2.0; // rad/s

        // Lateral PID (strafe to center tag in frame)
        public static final double kLatP = 0.8;
        public static final double kLatI = 0.0;
        public static final double kLatD = 0.05;
        public static final double kLatToleranceMeters = 0.03;
        public static final double kMaxLateralSpeed = 1.0; // m/s

        // Distance PID (forward/back to correct standoff)
        public static final double kDistP = 1.5;
        public static final double kDistI = 0.0;
        public static final double kDistD = 0.1;
        public static final double kDistToleranceMeters = 0.05;
        public static final double kMaxForwardSpeed = 1.0; // m/s

        // ==================== Tower Tag Pairs ====================
        /** Red upper tower tags */
        public static final int[] kRedUpperTowerTags = {13, 14};
        /** Red lower tower tags */
        public static final int[] kRedLowerTowerTags = {15, 16};
        /** Blue lower tower tags */
        public static final int[] kBlueLowerTowerTags = {29, 30};
        /** Blue upper tower tags */
        public static final int[] kBlueUpperTowerTags = {31, 32};

        // ==================== Tower Rung Centers ====================
        // Midpoints between each tag pair (field coordinates)
        public static final Translation2d kRedUpperRungCenter = new Translation2d(16.533, 7.19);
        public static final Translation2d kRedLowerRungCenter = new Translation2d(16.533, 4.11);
        public static final Translation2d kBlueLowerRungCenter = new Translation2d(0.008, 0.88);
        public static final Translation2d kBlueUpperRungCenter = new Translation2d(0.008, 3.96);

        // ==================== Approach Poses ====================
        // ~1.5m from wall, robot's back facing the wall
        // Red towers: wall at x=16.533, approach from x=15.033, heading=0 (back faces +X wall)
        // Blue towers: wall at x=0.008, approach from x=1.508, heading=180 (back faces -X wall)
        public static final Pose2d kRedUpperApproach = new Pose2d(15.033, 7.19, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedLowerApproach = new Pose2d(15.033, 4.11, Rotation2d.fromDegrees(0));
        public static final Pose2d kBlueLowerApproach = new Pose2d(1.508, 0.88, Rotation2d.fromDegrees(180));
        public static final Pose2d kBlueUpperApproach = new Pose2d(1.508, 3.96, Rotation2d.fromDegrees(180));

        // ==================== HP Station Poses ====================
        // Human player station starting positions (near alliance wall, facing field)
        public static final Pose2d kRedHPStation = new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(180));
        public static final Pose2d kBlueHPStation = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));

        // ==================== Helper Methods ====================

        /**
         * Gets the approach pose for the selected tower.
         * @param isRedAlliance true if red alliance
         * @param useUpperTower true for upper tower, false for lower
         * @return Approach Pose2d (~1.5m from wall, back facing wall)
         */
        public static Pose2d getApproachPose(boolean isRedAlliance, boolean useUpperTower) {
            if (isRedAlliance) {
                return useUpperTower ? kRedUpperApproach : kRedLowerApproach;
            } else {
                return useUpperTower ? kBlueUpperApproach : kBlueLowerApproach;
            }
        }

        /**
         * Gets the rung center for the selected tower.
         * @param isRedAlliance true if red alliance
         * @param useUpperTower true for upper tower, false for lower
         * @return Translation2d of the rung center on the field
         */
        public static Translation2d getRungCenter(boolean isRedAlliance, boolean useUpperTower) {
            if (isRedAlliance) {
                return useUpperTower ? kRedUpperRungCenter : kRedLowerRungCenter;
            } else {
                return useUpperTower ? kBlueUpperRungCenter : kBlueLowerRungCenter;
            }
        }

        /**
         * Gets the tower tag IDs for the selected tower.
         * @param isRedAlliance true if red alliance
         * @param useUpperTower true for upper tower, false for lower
         * @return Array of 2 AprilTag IDs for the tower
         */
        public static int[] getTowerTags(boolean isRedAlliance, boolean useUpperTower) {
            if (isRedAlliance) {
                return useUpperTower ? kRedUpperTowerTags : kRedLowerTowerTags;
            } else {
                return useUpperTower ? kBlueUpperTowerTags : kBlueLowerTowerTags;
            }
        }

        /**
         * Gets the HP station pose for the alliance.
         * @param isRedAlliance true if red alliance
         * @return Pose2d at the human player station
         */
        public static Pose2d getHPStationPose(boolean isRedAlliance) {
            return isRedAlliance ? kRedHPStation : kBlueHPStation;
        }

        /**
         * Gets the target heading when the robot's back faces the wall.
         * Red towers are on the +X wall, so robot faces +X (heading=0) to have back face wall...
         * Actually: back faces +X when heading=0 means front faces +X. We need back facing wall.
         * Red wall at x=16.533: back must face +X → robot heading = 0° (front +X, back -X)...
         * Wait — back facing the wall means the back of the robot points toward the wall.
         * Red wall is at high X. Back points toward +X when heading = 180° (front faces -X).
         * Blue wall is at low X. Back points toward -X when heading = 0° (front faces +X).
         *
         * @param isRedAlliance true if red alliance
         * @return Target heading in radians for back-to-wall alignment
         */
        public static double getTargetHeadingRad(boolean isRedAlliance) {
            // Red: wall at +X side, back must face +X → heading = 180° (π)
            // Blue: wall at -X side, back must face -X → heading = 0° (0)
            return isRedAlliance ? Math.PI : 0.0;
        }
    }

    // ==================== [MechanismName]Constants ====================
    // Add new nested classes for each mechanism
    // Example:
    // public static final class IntakeConstants {
    //     public static final double kIntakeSpeed = 0.8;
    //     public static final double kOuttakeSpeed = -0.5;
    // }
}
