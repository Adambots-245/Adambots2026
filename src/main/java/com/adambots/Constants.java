// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

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
     * Constants for the swerve drive system.
     * Speed limits, gearing, and dimensions are configured in YAGSL JSON and PathPlanner settings.json.
     */
    public static final class DriveConstants {
        /** Joystick deadzone for translational movement */
        public static final double kDeadzone = 0.05;

        // PathPlanner path-following PID (corrects position/heading error during auto)
        // These are NOT the same as YAGSL motor-level PIDs in pidfproperties.json.
        // Start at 5.0/0/0 and tune on the field — see SwerveConfig javadoc for tips.
        public static final double kAutoTranslationP = 5.0;
        public static final double kAutoTranslationI = 0.0;
        public static final double kAutoTranslationD = 0.0;

        public static final double kAutoRotationP = 5.0;
        public static final double kAutoRotationI = 0.0;
        public static final double kAutoRotationD = 0.0;
    }

    // ==================== ShooterConstants ====================
    /**
     * Constants for the flywheel shooter subsystem.
     * Tested PID values from Subsystem/Shooter branch test board.
     */
    public static final class ShooterConstants {
        // ==================== Flywheel Motor Specs ====================
        public static final double kMotorFreeSpeedRPS = 100.0; // Kraken X60: 6000 RPM = 100 RPS
        public static final double kNominalVoltage = 12.0;

        // Set to -1.0 to reverse flywheel direction (workaround for setInverted issue)
        public static final double kFlywheelDirection = -1.0;

        // ==================== Flywheel PID (tested on test board) ====================
        public static final double kFlywheelP = 0.1;
        public static final double kFlywheelI = 0;
        public static final double kFlywheelD = 0;
        public static final double kFlywheelFF = kNominalVoltage / kMotorFreeSpeedRPS; // 0.12 V/RPS

        public static final double kFlywheelToleranceRPS = 2.0;

        // ==================== Current Limits ====================
        public static final double kFlywheelStallCurrentLimit = 40.0;
        public static final double kFlywheelFreeCurrentLimit = 60.0;

        // ==================== Interpolation Table ====================
        // distance (meters) -> RPS, tuned on the field
        public static final double[][] kDefaultInterpolationTable = {
            {2.0, 45.0},
            {2.5, 46.5},
            {3.0, 48.0},
            {4.0, 54.0},
            {5.0, 60.0}
        };
    }

    // ==================== TurretConstants ====================
    /**
     * Constants for the turret position-controlled subsystem.
     * Tested PID values from Subsystem/Shooter branch test board.
     */
    public static final class TurretConstants {
        // ==================== Turret PID (tested on test board) ====================
        public static final double kTurretP = 0.75;
        public static final double kTurretI = 0;
        public static final double kTurretD = 0;
        public static final double kTurretFF = 0.2;

        // ==================== Turret Mechanical ====================
        // WCP GreyT Turret: 200-tooth ring gear / 18-tooth pinion
        public static final double kTurretGearRatio = 200.0 / 18.0;

        // 180° total range centered at 0° (±90°)
        public static final double kTurretMinDegrees = -90.0;
        public static final double kTurretMaxDegrees = 90.0;

        // Soft limit rotations derived from degrees/360 * gearRatio
        public static final double kTurretForwardLimit = (kTurretMaxDegrees / 360.0) * kTurretGearRatio;
        public static final double kTurretReverseLimit = (kTurretMinDegrees / 360.0) * kTurretGearRatio;

        public static final double kTurretManualSpeed = 0.3;

        // ==================== Current Limits ====================
        public static final double kTurretStallCurrentLimit = 20.0;
        public static final double kTurretFreeCurrentLimit = 40.0;
    }

    // ==================== TurretTrackingConstants ====================
    public static final class TurretTrackingConstants {
        /** Motor duty cycle for turret scan sweep */
        public static final double kScanSpeed = 0.15;
        /** Degrees tolerance to consider turret "on target" for tracking */
        public static final double kTrackingToleranceDeg = 2.0;
        /** Seconds to wait after losing target before starting scan */
        public static final double kLostTargetTimeoutSec = 0.5;
    }

    // ==================== HopperConstants ====================
    public static final class HopperConstants {
        public static final double kHopperSpeed = 0.5;
        public static final double kUptakeSpeed = 0.5;
        public static final double kDetectionRange = 2.0; // cm
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
}
