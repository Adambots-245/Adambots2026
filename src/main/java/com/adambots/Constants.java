// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

        /** Back-left ArduCam OV9281 (on back-left swerve module, facing backward) */
        public static final String kBackLeftCameraName = "back_left";

        /** Back-right ArduCam OV9281 (on back-right swerve module, facing backward) */
        public static final String kBackRightCameraName = "back_right";

        /** Shooter LifeCam HD-3000 (on top of shooter, facing forward) */
        public static final String kShooterCameraName = "shooter_cam";

        // ==================== Camera Positions ====================
        // All measurements are in METERS from the robot center to the camera lens.
        // Robot center = the point on the floor exactly between the four swerve modules.
        //
        // How to measure each axis:
        //   X: Measure from robot center toward the front (+) or back (-) of the robot
        //      to the camera lens. Use a tape measure along the floor, then project up.
        //      Example: camera is 11in behind center → X = -0.28m
        //
        //   Y: Measure from robot center toward the left (+) or right (-) side.
        //      Stand behind the robot looking forward: left is positive.
        //      Example: camera is 11in to the left → Y = +0.28m
        //
        //   Z: Measure from the floor straight up to the camera lens.
        //      Example: camera sits 8.5in above the ground → Z = 0.22m
        //
        //   Pitch: Tilt angle of the camera up/down in degrees.
        //      0 = level, negative = tilted down, positive = tilted up.
        //      Use a phone inclinometer app on the camera to measure.
        //
        //   Yaw: Direction the camera faces in degrees.
        //      0 = forward, 90 = left, 180 = backward, -90 = right.
        //
        // Back swerve modules are at (-0.28m, ±0.28m) from center per swerve config.
        // TODO: Fine-tune all positions with actual measurements from the robot

        // Back-Left ArduCam (on top of back-left swerve module, facing backward)
        public static final double kBackLeftCameraX = -0.28;   // back of robot
        public static final double kBackLeftCameraY = 0.28;    // left side
        public static final double kBackLeftCameraZ = 0.22;    // on top of module (~8.5in up)
        public static final double kBackLeftCameraRoll = 0.0;
        public static final double kBackLeftCameraPitch = 15.0;  // slightly up to see AprilTags
        public static final double kBackLeftCameraYaw = 180.0;   // facing backward

        // Back-Right ArduCam (on top of back-right swerve module, facing backward)
        public static final double kBackRightCameraX = -0.28;  // back of robot
        public static final double kBackRightCameraY = -0.28;  // right side
        public static final double kBackRightCameraZ = 0.22;   // on top of module (~8.5in up)
        public static final double kBackRightCameraRoll = 0.0;
        public static final double kBackRightCameraPitch = 15.0;  // slightly up to see AprilTags
        public static final double kBackRightCameraYaw = 180.0;   // facing backward

        // Shooter LifeCam (on top of shooter at back of robot, facing forward)
        public static final double kShooterCameraX = -0.20;   // behind center (shooter is at back)
        public static final double kShooterCameraY = 0.0;      // centered
        public static final double kShooterCameraZ = 0.50;     // on top of shooter (~20in up)
        public static final double kShooterCameraRoll = 0.0;
        public static final double kShooterCameraPitch = 0.0;  // level
        public static final double kShooterCameraYaw = 0.0;    // facing forward

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

        /** Maximum distance to recognize AprilTags for odometry cameras (meters) */
        public static final double kOdomMaxTagDistance = 4.0;

        /** Maximum distance to recognize AprilTags for alignment camera (meters) */
        public static final double kAlignMaxTagDistance = 6.0;

        // ==================== AprilTag Groups ====================
        // Game-specific tag groups for filtering and detection
        // Use with vision.hasID() or allowedTags() in VisionConfigBuilder
        // Reference: plans/vision-implementation.md for full tag layout

        // TODO: Verify tag IDs match 2026 REBUILT field layout

        /** Red alliance HUB tags - primary targets for scoring */
        public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

        /** Blue alliance HUB tags - primary targets for scoring */
        public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

        /** Red alliance TOWER BACKBOARD tags (for climb alignment and pose estimation) */
        public static final int[] kRedTowerTags = {15, 16};

        /** Blue alliance TOWER BACKBOARD tags (for climb alignment and pose estimation) */
        public static final int[] kBlueTowerTags = {31, 32};

        /** Red alliance OUTPOST tags (previously lumped with tower) */
        public static final int[] kRedOutpostTags = {13, 14};

        /** Blue alliance OUTPOST tags (previously lumped with tower) */
        public static final int[] kBlueOutpostTags = {29, 30};

        /** Red alliance TRENCH tags */
        public static final int[] kRedTrenchTags = {1, 6, 7, 12};

        /** Blue alliance TRENCH tags */
        public static final int[] kBlueTrenchTags = {17, 22, 23, 28};

        /** Max distance to consider a target valid (meters) */
        public static final double kMaxDistanceMeters = 8.0;

        /** Vision mode: 0 = Camera-only, 1 = Pose-only, 2 = Hybrid (camera primary, pose fallback) */
        public static final int kVisionMode = 2;

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
         * Gets the OUTPOST tags for the current alliance.
         * @param alliance The current alliance from DriverStation
         * @return Array of OUTPOST tag IDs for the alliance
         */
        public static int[] getOutpostTags(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? kRedOutpostTags : kBlueOutpostTags;
        }

        /**
         * Gets the OUTPOST tags for the current alliance.
         * @param isRedAlliance true if on red alliance, false if on blue
         * @return Array of OUTPOST tag IDs for the alliance
         */
        public static int[] getOutpostTags(boolean isRedAlliance) {
            return isRedAlliance ? kRedOutpostTags : kBlueOutpostTags;
        }

        /**
         * Gets the TRENCH tags for the current alliance.
         * @param alliance The current alliance from DriverStation
         * @return Array of TRENCH tag IDs for the alliance
         */
        public static int[] getTrenchTags(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            return alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? kRedTrenchTags : kBlueTrenchTags;
        }

        /**
         * Gets the TRENCH tags for the current alliance.
         * Use with Utils.isOnRedAlliance() for cleaner code.
         * @param isRedAlliance true if on red alliance, false if on blue
         * @return Array of TRENCH tag IDs for the alliance
         */
        public static int[] getTrenchTags(boolean isRedAlliance) {
            return isRedAlliance ? kRedTrenchTags : kBlueTrenchTags;
        }

        // ==================== Network Configuration ====================

        /** PhotonVision coprocessor IP address (OrangePi) */
        public static final String kPhotonVisionIP = "10.2.45.11";

        /** PhotonVision web dashboard port */
        public static final int kPhotonVisionPort = 5800;

        // ==================== Climb Alignment Constants ====================

        /**
         * Lateral offset from tag-pair centroid to tower opening centerline (meters).
         * The two tower backboard tags are NOT centered on the opening — their centroid
         * is shifted ~8.4" toward one upright. This offset corrects for that.
         * Positive = toward field +Y for red, –Y for blue. Tune on-field if needed.
         */
        public static final double kTowerTagLateralOffsetMeters = 0.213;

        /**
         * Distance from the alliance wall to the robot center when positioned for climb (meters).
         * TODO: Measure on field — this depends on elevator extension and U-frame geometry.
         */
        public static final double kClimbWallDistanceMeters = 0.0; // TBD — measure on field

        /**
         * Robot heading when aligned for climb (radians).
         * Robot backs into the tower, so heading faces away from the wall.
         * TODO: Set based on alliance wall orientation (0 or π).
         */
        public static final double kClimbHeadingRad = 0.0; // TBD — set per alliance wall

        /**
         * Precomputed climb poses (robot center) derived from field drawings + offsets.
         * TODO: Compute from tower coordinates, kTowerTagLateralOffsetMeters, and kClimbWallDistanceMeters.
         * For now these are placeholders — fill in once measurements are taken.
         */
        // public static final Pose2d kRedClimbPose = new Pose2d(X, Y, Rotation2d.fromDegrees(heading));
        // public static final Pose2d kBlueClimbPose = new Pose2d(X, Y, Rotation2d.fromDegrees(heading));
    }

    // ==================== Intake Constants ====================
    public static final class IntakeConstants {
        // Onboard PID gains (TalonFX 1kHz loop, MotionMagicVoltage)
        // All feedforward gains are in Volts (since we use voltage-based control)
        // Tuning order: kG first (hold horizontal), then kS, kP, kD
        public static final double kArmP = 4.8;    // Volts per rotation of error
        public static final double kArmI = 0.0;     // Volts per rotation*second of error (almost never needed with proper kG)
        public static final double kArmD = 0.1;     // Volts per rotation/second of error (damping)
        public static final double kArmKV = 0.12;   // Volts per rotation/second of velocity
        public static final double kArmKS = 0.25;   // Volts to overcome static friction
        public static final double kArmKA = 0.01;   // Volts per rotation/second^2 of acceleration
        public static final double kArmKG = 0.35;   // Volts to hold arm horizontal (tune with Phoenix Tuner X)

        // Motion Magic profile constraints
        public static final double kArmCruiseVelocity = 2.0;  // rotations per second
        public static final double kArmAcceleration = 1.0;     // rotations per second^2
        public static final double kArmJerk = 0.0;             // 0 = no jerk limiting

        // Intake arm gear ratio — two-stage reduction (motor → mechanism)
        // Stage 1: Planetary gearbox on Minion motor
        // Stage 2: Belt-driven pulley from gearbox output to arm pivot
        // Total ratio = stage1 * stage2 (motor rotations per mechanism rotation)
        public static final double kArmPlanetaryRatio = 20.0;  // e.g., 5.0 for 5:1 planetary
        public static final double kArmBeltRatio = 2.0;        // e.g., 2.0 for 36T:18T belt
        public static final double kArmTotalGearRatio = kArmPlanetaryRatio * kArmBeltRatio;

        public static final double kLowSpeed = 0.3;
        public static final double kHighSpeed = 0.5;

        public static final double kArmRaisedPosition = 0.0;   // motor rotations when arm is raised (retracted, home)
        public static final double kArmLoweredPosition = 0.25; // motor rotations when arm is lowered (deployed)

        // Arm motor current limits
        public static final int kArmStatorCurrentLimit = 40;  // stator amps (torque limiting)
        public static final int kArmSupplyCurrentLimit = 30;  // supply amps (must be ≤ PDH breaker)
    }

    /**
     * Simulation constants — approximate values for code testing, not hardware tuning.
     */
    public static final class SimConstants {
        public static final double kArmLengthMeters = 0.4;
        public static final double kArmMassKg = 2.0;
        public static final double kArmMinAngleRad = Math.toRadians(-10);   // slightly below horizontal
        public static final double kArmMaxAngleRad = Math.toRadians(100);   // just past vertical
        public static final double kArmStartAngleRad = Math.toRadians(0);   // start horizontal (lowered)

        // Estimated physical gear ratio for sim only.
        // Inferred from: 7.0 motor rotations ≈ 90° arm travel → ratio ≈ 28:1
        // This is separate from IntakeConstants.kArmTotalGearRatio (which controls
        // sensorToMechanismRatio and must stay 1.0 until position targets are recalibrated).
        public static final double kSimGearRatio = 28.0;
    }


    // ==================== [MechanismName]Constants ====================
    // Add new nested classes for each mechanism
    // Example:
    // public static final class IntakeConstants {
    //     public static final double kIntakeSpeed = 0.8;
    //     public static final double kOuttakeSpeed = -0.5;
    // }
}
