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

    // ==================== Battery Tracking ====================

    /**
     * Battery identifier for the pack installed for the current deploy.
     *
     * <p><b>CHANGE BEFORE EACH MATCH</b> — label each physical battery with a
     * number (1, 2, 3…) and set this constant to the installed battery's number
     * prior to running {@code ./gradlew deploy}. Logged as metadata + ESSENTIAL
     * signal so post-match analysis can correlate voltage sag / brownouts
     * (e.g. the MICMP1 Q19/Q58 packs) to a specific physical battery.
     *
     * <p>Default {@code 0} means "unset" — treat those logs with caution.
     */
    public static final int BATTERY_ID = 0;

    // ==================== Operating Mode ====================

    /**
     * Robot operating mode. Drives {@link com.adambots.Robot}'s AdvantageKit receiver
     * configuration:
     * <ul>
     *   <li>{@link Mode#REAL} — running on a real roboRIO. WPILOGWriter + NT4Publisher active.</li>
     *   <li>{@link Mode#SIM} — desktop simulation. NT4Publisher only (no disk).</li>
     *   <li>{@link Mode#REPLAY} — re-run a recorded log through the code for debugging.
     *       Uses WPILOGReader + WPILOGWriter with {@code _sim} suffix.</li>
     * </ul>
     */
    public enum Mode { REAL, SIM, REPLAY }

    /** Set per deploy target. REAL for robot, SIM for desktop, REPLAY for log replay. */
    public static final Mode MODE = Mode.REAL;

    // ==================== Log Level ====================

    /**
     * Verbosity floor for {@link com.adambots.logging.LogUtil#log}. Compile-time.
     * <ul>
     *   <li>{@link com.adambots.logging.LogUtil.Level#ESSENTIAL} — quals/comp. ~25 signals.</li>
     *   <li>{@link com.adambots.logging.LogUtil.Level#DIAGNOSTIC} — practice / post-match tuning.
     *       Adds vision pipeline internals, per-module swerve, command durations.</li>
     *   <li>{@link com.adambots.logging.LogUtil.Level#DEBUG} — bench only. Adds high-rate
     *       tuning data (lead-comp math, PID internals).</li>
     * </ul>
     * A change requires {@code ./gradlew clean deploy} to take effect (static final inline).
     */
    public static final com.adambots.logging.LogUtil.Level LOG_LEVEL =
        com.adambots.logging.LogUtil.Level.DIAGNOSTIC;

    // ==================== Tuning / Shuffleboard (separate from logging) ====================

    /** Set false for competition — disables all Shuffleboard tunables and their NT reads. */
    public static final boolean TUNING_ENABLED = true;

    // Per-tab enables — only effective when TUNING_ENABLED is true.
    // Set individual flags to false to reduce bandwidth while tuning a specific subsystem.
    public static final boolean SHOOTER_TAB  = TUNING_ENABLED && true;
    public static final boolean SWERVE_TAB   = TUNING_ENABLED && false;
    public static final boolean CLIMBER_TAB  = TUNING_ENABLED && false;
    public static final boolean COMMANDS_TAB = TUNING_ENABLED && false;
    public static final boolean VISION_TAB   = TUNING_ENABLED && false;
    public static final boolean INTAKE_TAB   = TUNING_ENABLED && false;
    public static final boolean HOPPER_TAB   = TUNING_ENABLED && false;
    public static final boolean TURRET_TAB  = TUNING_ENABLED && true;

    /**
     * @deprecated Replaced by {@link #LOG_LEVEL} — current logging is now part of ESSENTIAL.
     *   Remove after all call sites migrate to {@link com.adambots.logging.LogUtil}.
     */
    @Deprecated
    public static final boolean CURRENT_LOGGING = true;

    /**
     * @deprecated Replaced by {@link #LOG_LEVEL} — set to {@code DIAGNOSTIC} or higher for
     *   the same effect. Remove after all call sites migrate to {@link com.adambots.logging.LogUtil}.
     */
    @Deprecated
    public static final boolean LOGGING_ENABLED = false;

    /** Shuffleboard visible grid size — tweak to match your screen/layout. */
    public static final int kShuffleboardCols = 10;
    public static final int kShuffleboardRows = 5;

    /** LED strip configuration — update kLEDStripLength once strip is wired. */
    public static final int kLEDStripLength = 105;
    public static final int kProgressStartIndex = 8;    // first external LED (after 8 onboard)
    public static final int kProgressLEDCount = 97;    // LEDs used for the progress bar

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
        public static final double kAutoTranslationP = 2.5;
        public static final double kAutoTranslationI = 0.0;
        public static final double kAutoTranslationD = 0.0;

        public static final double kAutoRotationP = 2.5;
        public static final double kAutoRotationI = 0.0;
        public static final double kAutoRotationD = 0.0;

        /** Max translation speed scale (0-1]. 0.8 = 80% of max chassis velocity */
        // Going above 90% may make turning at top speed sluggish
        public static final double kTranslationScale = 0.9;
    }

    // ==================== ShooterConstants ====================
    /**
     * Constants for the flywheel shooter subsystem.
     * Tested PID values from Subsystem/Shooter branch test board.
     */
    public static final class ShooterConstants {
        // ==================== Flywheel Motor Specs ====================
        public static final double kMotorFreeSpeedRPS = 96.4; // Kraken X60 FOC: 5784 RPM = 96.4 RPS
        public static final double kNominalVoltage = 12.0;

        // Set to -1.0 to reverse flywheel direction (workaround for setInverted issue)
        public static final double kFlywheelDirection = -1.0;

        // ==================== Flywheel PID (VelocityTorqueCurrentFOC — Pro) ====================
        // Units: kP in amps per RPS error, kFF not needed (torque mode).
        // Old voltage-mode gains (kP=0.35 V/RPS, kFF=0.12 V/RPS) don't apply.
        // Start at kP=5.0, tune on robot: increase if recovery is slow,
        // decrease if flywheel oscillates or draws excessive current.
        public static final double kFlywheelP = 5.0;   // was 0.35 (voltage mode)
        public static final double kFlywheelI = 0;
        public static final double kFlywheelD = 0;
        public static final double kFlywheelFF = 0;     // was 0.12 — torque mode doesn't need voltage FF

        public static final double kFlywheelToleranceRPS = 3.5;  // was 2.5 — shot boost adds 3 RPS above base target, causing at_speed chatter at 2.5

        /** Fixed RPS for mid-field lob shots (tune on field). */
        public static final double kLobShotRPS = 62.0;

        // ==================== Current Limits ====================
        public static final double kFlywheelStatorCurrentLimit = 40.0;
        public static final double kFlywheelSupplyCurrentLimit = 25.0;  // was 40 — power-budget calc peak 13.7A @ 4000 RPM, back-EMF margin to 25A; battery-friendlier during shooter+drive simultaneity

        // ==================== Interpolation Table ====================
        // distance (meters) -> RPS, tuned on the field
        public static final double[][] kDefaultInterpolationTable = {
            {1.5, 39.0 + 2},
            {2.0, 40.5 + 2},
            {2.5, 43.0 + 2},
            {3.0, 46.0 + 3},
            {4.0, 53.0 + 3},
            {5.0, 59.0 + 3}
        };

        public static final double kMinRPS = kDefaultInterpolationTable[0][1];
        public static final double kMaxRPS = kDefaultInterpolationTable[kDefaultInterpolationTable.length - 1][1];

        /** Idle pre-spin RPS — keeps flywheel warm for faster spin-up. */
        public static final double kIdleRPS = 20.0;

        /** Extra RPS added during feed to compensate for ball energy transfer to flywheel. */
        public static final double kShotBoostRPS = 3;

        // Shooting zone bounds — robot X position that defines "near the hub"
        public static final double kRedShootingZoneMinX = 12.0;   // Red hub at x≈12.0
        public static final double kBlueShootingZoneMaxX = 4.54;  // Blue hub at x≈4.54

        /** Enable chassis shake during shooting to settle balls into carousel. */
        public static final boolean kShakeEnabled = false;
        /** Rotational speed for chassis shake (rad/s). */
        public static final double kShakeRotSpeed = 1.5;
        /** Period of one full shake cycle (seconds). */
        public static final double kShakePeriodSeconds = 0.2;
    }

    // ==================== TurretConstants ====================
    /**
     * Constants for the turret position-controlled subsystem.
     * Tested PID values from Subsystem/Shooter branch test board.
     */
    public static final class TurretConstants {
        // ==================== Turret PID ====================
        // Minion motor: 3.17 N·m stall, 211A stall, 7704 RPM (128.4 RPS) free, 12V
        // Theoretical kV = 12V / 128.4 RPS = 0.094 — actual tuned value is higher
        // (0.135) because real-world friction/load requires more voltage per RPS.
        // kP: 20 × 0.031 rot/deg = 0.62V per degree error — enough to correct without overshoot
        // kD: kept low — Minion velocity signal is noisy, high D goes berserk
        // kS: static friction compensation — voltage needed to just barely start
        //     turning the turret from standstill. Helps the PID break through
        //     sticky spots (3D-printed gear mesh, cable tray friction) instead
        //     of buzzing at the friction breakaway boundary. Start at 0.25 and
        //     tune: too low = buzz remains, too high = turret "jumps" when
        //     correcting small errors.
        // PID gains — tuned for 4:1 planetary (44.4:1 total).
        // The planetary multiplies motor rotations per turret degree by 4×,
        // so kP should be ~4× lower than without the planetary to get the
        // same turret-level response. Start here and tune on the robot.
        public static final double kTurretP = 5.0;     // was 18 without planetary
        public static final double kTurretI = 0;
        public static final double kTurretD = 0.1;
        public static final double kTurretKV = 0.025;  // was 0.100 — 4× more motor rot per turret rot
        public static final double kTurretKS = 0.10;   // static friction compensation (Volts) — planetary reduces stiction
        public static final double kTurretKA = 0.0;    // accel feedforward (0 for now)
        public static final double kTurretKG = 0.0;    // gravity (0 — turret is horizontal)

        // ==================== Motion Magic Profile ====================
        // Simulation-optimized for tracking. Median correction is 0.8° turret —
        // all profiles are triangular (never reach cruise). Lower values produce
        // smoother motion for these tiny moves. Large slews (>15°) are rare.
        public static final double kTurretCruiseVelocity = 80.0;   // RPS at motor (simulation-optimized)
        public static final double kTurretAcceleration = 400.0;    // RPS/s at motor (simulation-optimized)
        public static final double kTurretJerk = 0.0;              // 0 = trapezoidal (no s-curve)

        // ==================== Turret Mechanical ====================
        // WCP GreyT Turret: 200-tooth ring gear.
        // Motor has a 4:1 planetary before a 20T pinion.
        // Pot has its own separate 18T pinion (no planetary).
        public static final double kTurretPotGearRatio = 200.0 / 18.0;            // pot → turret (18T pinion)
        public static final double kTurretMotorGearRatio = (200.0 / 20.0) * 4.0;  // motor → turret (20T pinion + 4:1 planetary)

        // ==================== Potentiometer Calibration ====================
        // The 10-turn potentiometer has its own 18T pinion on the 200T ring
        // gear (no planetary in its path). Calibrate by parking the turret
        // at each mechanical stop, reading "Pot Raw (deg)" on the Shooter
        // tab, and putting the value here.
        /** Pot reading (degrees) when turret is at 0° — determine empirically via dashboard */
        public static final double kTurretPotAtZeroDeg = 107;
        /** Pot reading (degrees) when turret is at max — determine empirically via dashboard */
        public static final double kTurretPotAtMaxDeg = 2660.0;

        /**
         * Turret physical range in degrees, derived from the pot endpoints and
         * the gear ratio. Must be computed — if hand-entered, it can disagree
         * with the motor-encoder-based {@code getTurretAngleDegrees()} readback
         * and the pot-based {@code getPotAngleDegrees()} will drift.
         *
         * <p>Derivation: the pot has its own 18T pinion (no planetary), so
         * pot travel / pot gear ratio = turret range.
         *
         * <p>{@code Math.abs(...)} keeps the range positive regardless of which
         * endpoint has the larger raw pot reading. Needed when the pot is wired
         * such that raw count <b>decreases</b> as the turret sweeps toward max
         * (since 2026-04, after pot replacement). {@code TurretSubsystem.getPotAngleDegrees()}
         * is already polarity-agnostic; only this range magnitude needed fixing.
         */
        public static final double kTurretMaxDegrees =
            Math.abs(kTurretPotAtMaxDeg - kTurretPotAtZeroDeg) / kTurretPotGearRatio;

        /** Turret angle (degrees) that faces straight ahead on the robot.
         *  Re-measure after any change to the pot calibration. */
        public static final double kTurretForwardDegrees = 88.0;

        /** Turret pivot offset from robot center (meters).
         *  X = forward/back (negative = behind center), Y = left/right.
         *  Measure from swerve module diagonal intersection to ring gear center.
         *  Using shooter camera position as proxy until measured. */
        public static final double kTurretPivotX = 0;  // 20cm behind center
        public static final double kTurretPivotY = 0;  // 12cm right of center (negative = right)

        /** Percent-output magnitude for manual jog (Turret Left/Right, D-pad E/W).
         *  0.15 ≈ 15% voltage. Adjust for feel — higher = faster jog, lower = finer. */
        public static final double kTurretJogPercent = 0.25;  // was 0.10 — need more voltage through 4:1 planetary

        /** Joystick deadband for proportional turret jog (integrated into auto-track). */
        public static final double kTurretJogDeadband = 0.10;
        /** Max percent output for joystick jog. Joystick input is squared for fine control. */
        public static final double kTurretJogMaxPercent = 0.35;

        /** Soft limit margin beyond [0, kTurretMaxDegrees], in turret degrees.
         *  Firmware cuts motor output if position drifts this far outside the
         *  calibrated range — safety rail for jog mode (percent output bypasses
         *  the software clamp in setTurretAngle). */
        public static final double kTurretSoftLimitMarginDeg = 6.0;

        // ==================== Current Limits ====================
        public static final double kTurretStatorCurrentLimit = 40.0;  // was 30 — raised for higher MM accel (500 RPS/s)
        public static final double kTurretSupplyCurrentLimit = 25.0;  // was 20 — headroom for tracking corrections
    }

    // ==================== TurretTrackingConstants ====================
    public static final class TurretTrackingConstants {
        /** Degrees tolerance for isAtTarget trigger (used by shoot commands). */
        public static final double kTrackingToleranceDeg = 2.5;
        /** Degrees margin from turret limits before reversing scan direction */
        public static final double kScanMarginDeg = 15.0;
        /** Anticipation time for angular velocity feedforward (seconds).
         *  Turret leads the setpoint by robotAngVel × this value to compensate for rotation. */
        public static final double kAngularVelLeadTime = 0.03;

        /**
         * Projectile time-of-flight (seconds) for <b>translational</b> lead compensation.
         * Shifts the aim point by {@code -fieldVelocity × kShotLeadTimeSec} so the turret
         * aims ahead of the hub to account for the robot's drift during the fuel's flight.
         *
         * <p><b>Zero disables it</b> — the aim point reduces exactly to the static hub
         * center, matching pre-lead behavior. Only the 5-arg
         * {@code poseTrackCommand(..., fieldVelSupplier, ...)} overload consumes this
         * value; the legacy 4-arg signature is unaffected.
         *
         * <p>Tuning: start at 0.15–0.20s for FUEL at typical scoring ranges (3–4 m,
         * exit speed ~15 m/s → TOF ≈ 0.2s). Increase if shots trail behind when
         * strafing, decrease if they lead too far. Bench-test with static hub first.
         */
        public static final double kShotLeadTimeSec = 0.0;
    }

    // ==================== HopperConstants ====================
    public static final class HopperConstants {
        public static final double kHopperSpeed = 0.40;  // was 0.30 — more torque to prevent jam stalls
        public static final double kUptakeSpeed = 0.60;

        // Jam detection (set kJamDetectionEnabled = false to disable entirely)
        public static final boolean kJamDetectionEnabled = true;
        public static final double kJamVelocityThreshold = 0.5;  // RPS — true stall (near zero)
        public static final double kJamStallDuration = 0.3;       // seconds velocity must stay below threshold to confirm jam
        public static final double kJamReverseDuration = 0.5;     // seconds to reverse when jam confirmed
        public static final double kJamGracePeriod = 0.50;        // was 0.25 — longer grace prevents buzz loop re-trigger

        // Current limits
        public static final double kHopperStatorCurrentLimit = 70.0;
        public static final double kHopperSupplyCurrentLimit = 50.0;
        public static final double kUptakeStatorCurrentLimit = 60.0;
        public static final double kUptakeSupplyCurrentLimit = 40.0;
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

        /** Shooter LifeCam HD-3000 (on top of shooter, facing forward) */
        public static final String kFrontCameraName = "forward_camera";


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
        // Camera positions measured from robot

        // Back-Left ArduCam (on top of back-left swerve module, facing backward)
         // Back-Left ArduCam (on top of back-left swerve module, facing backward)
        public static final double kBackLeftCameraX = -0.32;   // back of robot
        public static final double kBackLeftCameraY = 0.27;    // left side
        public static final double kBackLeftCameraZ = 0.18;    // on top of module (~8.5in up)
        public static final double kBackLeftCameraRoll = 0.0;
        public static final double kBackLeftCameraPitch = 22.5;  // slightly up to see AprilTags
        public static final double kBackLeftCameraYaw = 170.0;   // facing backward

        // Back-Right ArduCam (on top of back-right swerve module, facing backward)
        public static final double kBackRightCameraX = -0.33;  // back of robot
        public static final double kBackRightCameraY = -0.27;  // right side
        public static final double kBackRightCameraZ = 0.18;   // on top of module (~8.5in up)
        public static final double kBackRightCameraRoll = 0.0;
        public static final double kBackRightCameraPitch = 22.5;  // slightly up to see AprilTags
        public static final double kBackRightCameraYaw = -170.0;  // facing backward

        // Shooter LifeCam (on top of shooter at back of robot, facing forward)
        public static final double kShooterCameraX = -0.20;   // behind center (shooter is at back)
        public static final double kShooterCameraY = 0.0;      // centered
        public static final double kShooterCameraZ = 0.53;     // on top of shooter (~20in up)
        public static final double kShooterCameraRoll = 0.0;
        public static final double kShooterCameraPitch = 27.0;  // pitched up to see hub tags
        public static final double kShooterCameraYaw = 0.0;     // facing forward

        // Shooter LifeCam (on top of climb, facing forward)
        public static final double kFrontCameraX = -0.155;   // behind center (climb is at back)
        public static final double kFrontCameraY = 0.13;
        public static final double kFrontCameraZ = 0.48;     // on top of climb
        public static final double kFrontCameraRoll = 0.0;
        public static final double kFrontCameraPitch = 0.0;  // level
        public static final double kFrontCameraYaw = 0.0;     // facing forward


        // ==================== Pose Estimation Parameters ====================

        /**
         * Ambiguity threshold for rejecting poses (0.0 to 1.0).
         * Lower = more strict, rejects more ambiguous poses.
         * Raised to 0.8 — LifeCam at 3-4m hub distance routinely reports 0.3-0.8.
         */
        public static final double kAmbiguityThreshold = 0.8;

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

        /** Minimum tag image area (% of frame) for odometry cameras.
         *  Fuzzy distant tags report wrong PnP distances, so area-based filtering
         *  is more reliable than distance. A 6" tag at ~4m occupies ~0.5-1.0%.
         *  Start at 0.5%, increase if mid-field pose is still unstable. */
        public static final double kOdomMinTagAreaPercent = 0.5;

        /** Maximum distance to recognize AprilTags for alignment camera (meters) */
        public static final double kAlignMaxTagDistance = 6.0;

        /** Maximum disagreement between vision pose and odometry pose (meters).
         *  Vision measurements further than this from the current odom estimate are
         *  rejected as outliers (e.g. bad PnP from fuzzy distant tags). */
        public static final double kMaxOdomVisionDisagreement = 2.0;

        // ==================== AprilTag Groups ====================
        // Game-specific tag groups for filtering and detection
        // Use with vision.hasID() or allowedTags() in VisionConfigBuilder
        // Reference: plans/vision-implementation.md for full tag layout

        /** Red alliance HUB tags - primary targets for scoring */
        public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

        /** Blue alliance HUB tags - primary targets for scoring */
        public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

        /** Hardcoded hub center coordinates from official WPILib 2026 field layout.
         *  Derived from geometric center of all hub tag positions in 2026-rebuilt-welded.json.
         *  Using hardcoded values eliminates field-to-field tag placement variation. */
        public static final double kRedHubCenterX = 12.004 + 0.1; // add 0.1 to shift the shooting alignment.
        public static final double kRedHubCenterY = 4.035;
        public static final double kBlueHubCenterX = 4.537;
        public static final double kBlueHubCenterY = 4.035;

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

        /** Seconds with no camera results before flagging camera as offline. */
        public static final double kCameraOfflineThresholdSeconds = 2.0;

        /** Vision mode: 0 = Camera-only, 1 = Pose-only, 2 = Hybrid (camera primary, pose fallback), 3 = Blended (weighted avg) */
        public static final int kVisionMode = 0;

        /** Blend weight for mode 3: fraction of camera vs pose. 0.6 = 60% camera, 40% pose. */
        public static final double kVisionBlendWeight = 0.6;
        /** Max disagreement (degrees) between camera and pose before blend falls back to camera-only. */
        public static final double kBlendDisagreementThreshold = 20.0;

        /** Vision std dev scaling per m/s of robot speed. At 2 m/s: stdDevs *= 1 + 2*1.0 = 3x. */
        public static final double kVisionSpeedScaling = 1.0;

        // ==================== Vision Filtering ====================
        /** Exponential weighted average alpha — fraction of new measurement per frame.
         *  0.04 = 4% new, 96% prior (example: Team 6328). Higher = more responsive but noisier. */
        public static final double kVisionAlpha = 0.04;

        // ==================== Hub Visibility Holdoff ====================
        /** Max value for charge/decay holdoff counter.
         *  At 50, a full drain at -1/frame takes 1 second — survives long gaps between detections. */
        public static final int kCamCounterMax = 50;
        /** Charge rate per valid camera frame. With ~15% detection rate,
         *  charge=10 × 3 = 30 vs drain=1 × 22 = 22 → net +8 per 25-frame window. */
        public static final int kCamChargeRate = 10;
    }

    // ==================== ClimberConstants ====================
    public static final class ClimberConstants {
        public static final double kClimbSpeed = 0.8;
        public static final double kExtendSpeed = 0.5;
        public static final double kLowerSpeed = 0.2;  // slow extend under gravity
        public static final double kStatorCurrentLimit = 60.0;
        public static final double kSupplyCurrentLimit = 40.0;
    }

    // ==================== Intake Constants ====================
    public static final class IntakeConstants {
        // Onboard PID gains (TalonFX 1kHz loop, MotionMagicVoltage)
        // All feedforward gains are in Volts (since we use voltage-based control)
        // Tuning order: kG first (hold horizontal), then kS, kP, kD
        // Reduced for lighter roller intake (was wheels) — tune from these starting points.
        public static final double kArmP = 35;     // Volts per rotation of error 
        public static final double kArmI = 0.0;    // Volts per rotation*second of error (almost never needed with proper kG)
        public static final double kArmD = 0.08;   // Volts per rotation/second of error 
        public static final double kArmKV = 0.12;  // Volts per rotation/second of velocity (motor characteristic, unchanged)
        public static final double kArmKS = 0.20;  // Volts to overcome static friction 
        public static final double kArmKA = 0.005; // Volts per rotation/second^2 of acceleration 
        public static final double kArmKG = 0.20;  // Volts to hold arm horizontal 

        // Motion Magic profile constraints
        public static final double kArmCruiseVelocity = 6.0;  // rotations per second
        public static final double kArmAcceleration = 3.0;     // rotations per second^2
        public static final double kArmJerk = 0.0;             // 0 = no jerk limiting

        // Intake arm gear ratio — two-stage reduction (motor → mechanism)
        // Stage 1: Planetary gearbox on Minion motor
        // Stage 2: Belt-driven pulley from gearbox output to arm pivot
        // Total ratio = stage1 * stage2 (motor rotations per mechanism rotation)
        public static final double kArmPlanetaryRatio = 20.0;  // e.g., 5.0 for 5:1 planetary
        public static final double kArmBeltRatio = 2.0;        // e.g., 2.0 for 36T:18T belt
        public static final double kArmTotalGearRatio = kArmPlanetaryRatio * kArmBeltRatio;

        public static final double kIntakeSpeed = 0.80;

        public static final double kArmRaisedPosition = 193.0;   // throughbore degrees when arm is raised (retracted) — CALIBRATE
        public static final double kArmLoweredPosition = 100.0; // throughbore degrees when arm is lowered (deployed) — CALIBRATE
        // Bop positions: absolute throughbore degrees, captured the same way as
        // lowered/raised. Park the arm where you want each bop endpoint, read
        // "Arm Encoder (deg)" on the dashboard, put the value here — CALIBRATE.
        public static final double kBopBottomPosition = 105.0;  // bop oscillation low end
        public static final double kBopTopPosition    = 150.0;  // bop oscillation high end
        /** Optional dwell time at the bottom position before going back up.
         *  0.0 = no dwell (bop as fast as the arm can move). Increase to slow down bop. */
        public static final double kBopDwellSeconds = 0.0;
        /** Position tolerance (degrees) for detecting arm arrival at bop endpoints. */
        public static final double kBopPositionToleranceDeg = 3.0;

        /** Soft limit margin beyond lowered/raised, in degrees. The firmware
         *  cuts output if reported position drifts this far past either end
         *  of the [lowered, raised] range — a safety rail for runaways. */
        public static final double kArmSoftLimitMarginDeg = 20.0;
        public static final double kRollerRunningThreshold = 0.1; // RPS — above this = roller is spinning

        // Roller jam detection
        public static final double kRollerJamVelocityThreshold = 0.5; // RPS — below this = jammed
        public static final double kRollerJamReverseDuration = 0.3;   // seconds to reverse when jam detected
        public static final double kRollerJamGracePeriod = 0.25;      // seconds before jam detection activates

        // Roller motor current limits
        public static final int kRollerStatorCurrentLimit = 35;  // was 45 — reduced to prevent battery sag (54A peaks → 9.25V)
        public static final int kRollerSupplyCurrentLimit = 30;  // supply amps (must be ≤ PDH breaker)

        // Arm motor current limits
        public static final int kArmStatorCurrentLimit = 45;  // stator amps (torque limiting)
        public static final int kArmSupplyCurrentLimit = 40;  // supply amps (must be ≤ PDH breaker)

        /** Timeout for the PathPlanner "intake" named command (seconds). */
        public static final double kAutoIntakeTimeout = 3.0;
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
