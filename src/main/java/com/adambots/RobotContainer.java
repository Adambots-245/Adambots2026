// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import com.adambots.lib.utils.tuning.TuningResult;
import com.adambots.commands.IntakeCommands;
import com.adambots.commands.ShootCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Utils;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionSystem;
import edu.wpi.first.wpilibj.GenericHID;
import com.adambots.simulation.FuelProjectile;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.VisionSimSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 *
 * <p>This class uses Inversion of Control (IoC) - hardware devices are created in RobotMap
 * and passed to subsystems here. This allows for easier testing and hardware abstraction.
 */
@Logged
public class RobotContainer {

    // ==================== SECTION: SUBSYSTEMS ====================
    // Subsystems receive hardware devices from RobotMap (IoC pattern)

    /** Swerve drive subsystem - configured via YAGSL JSON files in deploy/swerve/ */
    private final SwerveSubsystem swerve;

    /** Intake subsystem for acquiring game pieces */
    private final IntakeSubsystem intake;

    /** Hopper subsystem for storing and staging game pieces */
    private final HopperSubsystem hopper;

    /** Shooter subsystem for launching game pieces */
    private final ShooterSubsystem shooter;

    /** Climber subsystem for end-game climbing */
    private final ClimberSubsystem climber;

    /** LED subsystem using CANdle for robot status indication */
    private final CANdleSubsystem leds;

    /** PhotonVision system for AprilTag detection and pose estimation */
    private VisionSystem vision;

    /** Vision simulation subsystem for PhotonVision sim */
    private final VisionSimSubsystem visionSim;

    // ==================== SECTION: AUTONOMOUS CHOOSER ====================
    /** Autonomous routine selector displayed on the dashboard */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     *
     * <p>Initialization order:
     * <ol>
     *   <li>Initialize subsystems with hardware from RobotMap (IoC)</li>
     *   <li>Setup vision</li>
     *   <li>Setup LEDs</li>
     *   <li>Setup default commands</li>
     *   <li>Configure button bindings</li>
     *   <li>Register PathPlanner named commands (MUST be before auto chooser)</li>
     *   <li>Setup autonomous chooser</li>
     *   <li>Setup dashboard</li>
     * </ol>
     */
    public RobotContainer() {
        // 1. Initialize swerve subsystem from YAGSL config
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

        // 2. Initialize subsystems with hardware from RobotMap (IoC pattern)
        intake = new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeSensor);
        hopper = new HopperSubsystem(RobotMap.kHopperCarouselMotor, RobotMap.kHopperUptakeMotor, RobotMap.kHopperSensor);
        shooter = new ShooterSubsystem(RobotMap.kShooterLeftMotor, RobotMap.kShooterRightMotor, RobotMap.kShooterTurretMotor);
        climber = new ClimberSubsystem(RobotMap.kClimberLeftMotor, RobotMap.kClimberRightMotor,
                                          RobotMap.kClimberLeftLimit, RobotMap.kClimberRightLimit);
        leds = new CANdleSubsystem(RobotMap.kCANdlePort);

        // Initialize vision simulation subsystem
        visionSim = new VisionSimSubsystem("shooter_camera");

        // 3. Setup vision
        configureVision();

        // 4. Setup LEDs
        configureLEDs();

        // 5. Setup default commands
        configureDefaultCommands();

        // 6. Configure button bindings
        configureButtonBindings();

        // 7. Register PathPlanner named commands (MUST be before auto chooser)
        configurePathPlannerCommands();

        // 8. Setup autonomous chooser
        configureAutoChooser();

        // 9. Setup dashboard
        configureDashboard();
    }

    // ==================== SECTION: VISION SETUP ====================
    /**
     * Configure PhotonVision cameras (decoupled from swerve).
     *
     * <p>New 3-step initialization pattern:
     * <ol>
     *   <li>Build VisionSystemConfig using VisionConfigBuilder</li>
     *   <li>Create PhotonVision instance with config, pose supplier, and Field2d</li>
     *   <li>Connect vision to swerve via swerve.setupVision()</li>
     * </ol>
     *
     * <p>Uses VisionConfigBuilder from AdambotsLib to configure cameras:
     * <ul>
     *   <li>addCamera(name) - Camera name in PhotonVision</li>
     *   <li>position(x, y, z) - Position relative to robot center using Distance units</li>
     *   <li>rotation(roll, pitch, yaw) - Camera angle using Angle units</li>
     *   <li>purpose(CameraPurpose) - ODOMETRY, ALIGNMENT, or BOTH</li>
     *   <li>singleTagStdDevs / multiTagStdDevs - Pose estimation trust levels</li>
     *   <li>maxTagDistance(Distance) - Maximum distance to recognize AprilTags</li>
     *   <li>allowedTags(int[]) - Filter which tags this camera processes</li>
     * </ul>
     *
     * TODO: Update camera names and positions for your robot
     */
    private void configureVision() {
        // TODO: Configure vision cameras using VisionConfigBuilder
        // import static edu.wpi.first.units.Units.*;
        // import com.adambots.lib.vision.*;
        // import com.adambots.lib.vision.config.*;
        // import com.adambots.lib.utils.Utils;
        //
        // Step 1: Build the configuration
        // VisionSystemConfig visionConfig = VisionConfigBuilder.create()
        //     .addCamera("left_odom")
        //         .position(Meters.of(0.3), Meters.of(0.3), Meters.of(0.2))
        //         .rotation(Degrees.of(0), Degrees.of(-15), Degrees.of(0))
        //         .purpose(CameraPurpose.ODOMETRY)
        //         .singleTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0))
        //         .multiTagStdDevs(Meters.of(0.2), Meters.of(0.2), Radians.of(0.5))
        //         .maxTagDistance(Meters.of(4.0))
        //         .done()
        //     .addCamera("right_odom")
        //         .position(Meters.of(0.3), Meters.of(-0.3), Meters.of(0.2))
        //         .rotation(Degrees.of(0), Degrees.of(-15), Degrees.of(0))
        //         .purpose(CameraPurpose.ODOMETRY)
        //         .singleTagStdDevs(Meters.of(0.5), Meters.of(0.5), Radians.of(1.0))
        //         .multiTagStdDevs(Meters.of(0.2), Meters.of(0.2), Radians.of(0.5))
        //         .maxTagDistance(Meters.of(4.0))
        //         .done()
        //     .addCamera("turret")
        //         .position(Meters.of(0.1), Meters.of(0), Meters.of(0.15))
        //         .rotation(Degrees.of(0), Degrees.of(0), Degrees.of(0))
        //         .purpose(CameraPurpose.ALIGNMENT)
        //         .maxTagDistance(Meters.of(6.0))
        //         .allowedTags(Constants.VisionConstants.getHubTags(Utils.isOnRedAlliance()))
        //         .done()
        //     .ambiguityThreshold(0.2)
        //     .build();
        //
        // Step 2: Create PhotonVision independently
        // vision = new PhotonVision(
        //     visionConfig,
        //     swerve::getPose,      // Supplier<Pose2d> for current robot pose
        //     swerve.getField()     // Field2d for visualization
        // );
        //
        // Step 3: Connect vision to swerve
        // swerve.setupVision(vision);
    }

    // ==================== SECTION: LED SETUP ====================
    /**
     * Configure CANdle LED subsystem.
     *
     * <p>Available commands from CANdleSubsystem:
     * <ul>
     *   <li>setColorCommand(Color) - Set solid color</li>
     *   <li>turnOffCommand() - Turn off LEDs</li>
     *   <li>allianceColorCommand() - Set to alliance color</li>
     *   <li>blinkCommand(Color, count) - Blink a color</li>
     *   <li>pulseCommand(Color, speed) - Pulse a color</li>
     *   <li>strobeCommand(Color, speed) - Strobe effect</li>
     *   <li>celebrateCommand() - Celebration animation</li>
     *   <li>warningCommand() - Yellow warning</li>
     *   <li>errorCommand() - Red error</li>
     *   <li>readyCommand() - Green ready</li>
     *   <li>busyCommand() - Blue busy</li>
     *   <li>disabledCommand() - Disabled state</li>
     * </ul>
     *
     * <p>Available triggers:
     * <ul>
     *   <li>isShowingColorTrigger(Color)</li>
     *   <li>isOffTrigger()</li>
     *   <li>isAnimatingTrigger()</li>
     * </ul>
     */
    private void configureLEDs() {
        // Set default LED command - show alliance color when enabled
        leds.setDefaultCommand(leds.allianceColorCommand());

        // TODO: Add LED state bindings
        // Example: Show green when game piece detected
        // intake.hasGamePieceTrigger().onTrue(leds.setColorCommand(Color.kGreen));
        // intake.hasGamePieceTrigger().onFalse(leds.allianceColorCommand());

        // Example: Show ready when shooter is at speed
        // shooter.isAtSpeedTrigger().onTrue(leds.readyCommand());
    }

    // ==================== SECTION: DEFAULT COMMANDS ====================
    /**
     * Set the default command for each subsystem.
     * Default commands run whenever no other command is using that subsystem.
     */
    private void configureDefaultCommands() {
        // WORKAROUND: macOS Xbox controller axis mapping differs from WPILib's expected mapping.
        // WPILib's getRightX() reads axis 4, but on macOS axis 4 is a trigger (rests at -1.0).
        // Actual macOS mapping: axis 0=LeftX, 1=LeftY, 2=RightX, 3=RightY, 4/5=Triggers
        // TODO: Revert to Buttons.createForwardSupplier() etc. when using Extreme 3D Pro on real robot
        GenericHID driverHID = new GenericHID(RobotMap.kDriverJoystickPort);
        double deadzone = Constants.DriveConstants.kDeadzone;
        double rotDeadzone = Constants.DriveConstants.kRotationDeadzone;

        climber.setDefaultCommand(Commands.run(()->System.out.println("Test Message"), climber));
        swerve.setDefaultCommand(
            swerve.driveCommand(
                () -> -Buttons.applyDeadzone(driverHID.getRawAxis(1), deadzone),   // Left Y
                () -> -Buttons.applyDeadzone(driverHID.getRawAxis(0), deadzone),   // Left X
                () -> -Buttons.applyCubicCurve(Buttons.applyDeadzone(driverHID.getRawAxis(2), rotDeadzone)) // Right X (macOS) + cubic curve
            )
        );

        // TODO: Set default commands for other subsystems if needed
        // shooter.setDefaultCommand(shooter.idleCommand());
    }

    // ==================== SECTION: BUTTON BINDINGS ====================
    /**
     * Configure driver and operator button bindings.
     *
     * <p>Driver (Joystick - Extreme 3D Pro):
     * <ul>
     *   <li>Buttons.JoystickButton1 - Trigger button</li>
     *   <li>Buttons.JoystickButton2 - Thumb button</li>
     *   <li>Buttons.JoystickButton3-12 - Base buttons</li>
     * </ul>
     *
     * <p>Operator (Xbox Controller):
     * <ul>
     *   <li>Buttons.XboxAButton, XboxBButton, XboxXButton, XboxYButton - Face buttons</li>
     *   <li>Buttons.XboxLeftBumper, XboxRightBumper - Bumpers</li>
     *   <li>Buttons.XboxLeftTriggerButton, XboxRightTriggerButton - Triggers (as buttons)</li>
     *   <li>Buttons.XboxBackButton, XboxStartButton - Menu buttons</li>
     * </ul>
     */
    private void configureButtonBindings() {
        // === Driver Controls ===
        // Reset gyro heading - use Xbox Start button (was JoystickButton2 for Extreme 3D Pro)
        Buttons.XboxStartButton.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        // TODO: Add driver bindings
        // Buttons.JoystickButton1.whileTrue(swerve.lockWheelsCommand());

        // === Operator Controls ===
        // TODO: Add operator bindings for subsystems
        // Buttons.XboxAButton.whileTrue(IntakeCommands.intakeWhileHeldCommand(intake, hopper));
        // Buttons.XboxBButton.whileTrue(IntakeCommands.ejectCommand(intake, hopper));
        // Buttons.XboxXButton.onTrue(hopper.feedCommand());
        // Buttons.XboxYButton.onTrue(hopper.reverseUptakeCommand());
        // Buttons.XboxLeftBumper.whileTrue(ShootCommands.spinUpCommand(shooter));
        // Buttons.XboxRightBumper.onTrue(ShootCommands.shootCommand(shooter, hopper));

        // Climber controls (typically on operator controller)
        // Buttons.XboxDPadN.whileTrue(climber.extendCommand());
        // Buttons.XboxDPadS.whileTrue(climber.retractCommand());

        // === Cross-Subsystem Coordination ===
        // Bind commands to subsystem triggers for automatic coordination
        // Example: Turn LEDs green when game piece is detected
        // intake.hasGamePieceTrigger().onTrue(leds.setColorCommand(Color.kGreen));
        // intake.hasGamePieceTrigger().onFalse(leds.allianceColorCommand());

        // Example: Auto-index when intake detects game piece
        // intake.hasGamePieceTrigger().onTrue(hopper.indexOneCommand());
    }

    // ==================== SECTION: PATHPLANNER COMMANDS ====================
    /**
     * Register named commands for PathPlanner event markers and autos.
     *
     * <p>IMPORTANT: This method MUST be called BEFORE configureAutoChooser()
     * because PathPlanner needs all named commands registered before loading
     * any paths or autos that reference them.
     *
     * <p>These commands can be used in two ways:
     * <ul>
     *   <li><b>Event Markers</b>: Commands run during path following at specific waypoints
     *       (e.g., spin up shooter while driving to shooting position)</li>
     *   <li><b>Auto Chaining</b>: Commands run sequentially between path segments
     *       (e.g., shoot after arriving at position)</li>
     * </ul>
     *
     * <p>Best practices:
     * <ul>
     *   <li>Use event markers for actions that can happen while moving (spin up, deploy intake)</li>
     *   <li>Use auto chaining for actions requiring the robot to be stationary (shooting)</li>
     *   <li>Keep command names short and descriptive for PathPlanner GUI</li>
     * </ul>
     */
    private void configurePathPlannerCommands() {
        // ===== Shooter Commands =====
        // Spin up flywheel (use as event marker while driving to shooting position)
        NamedCommands.registerCommand("spinUp", shooter.spinUpCommand());

    }

    // ==================== SECTION: AUTONOMOUS SETUP ====================
    /**
     * Configure the autonomous command chooser with available auto routines.
     * Add PathPlanner autos and custom autonomous commands here.
     */
    private void configureAutoChooser() {
        // Default: do nothing
        autoChooser.setDefaultOption("None", Commands.none());

        // TODO: Add PathPlanner auto routines
        // autoChooser.addOption("Test Auto", swerve.getAutonomousCommand("Test"));
        // autoChooser.addOption("2 Piece Auto", swerve.getAutonomousCommand("2Piece"));
        // autoChooser.addOption("3 Piece Center", swerve.getAutonomousCommand("3PieceCenter"));

        // TODO: Add custom autonomous commands using AutoCommands factory
        // autoChooser.addOption("Score and Taxi", AutoCommands.scoreAndTaxiCommand(swerve, shooter, hopper, 2.0));
        // autoChooser.addOption("Score and Move", AutoCommands.scoreAndMoveCommand(swerve, shooter, hopper, new Pose2d()));

        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // ==================== SECTION: DASHBOARD ====================
    /**
     * Add telemetry and dashboard widgets.
     * This is called once during initialization.
     */
    private void configureDashboard() {
        // Swerve telemetry is handled by SwerveSubsystem

        // TODO: Add subsystem status indicators
        // SmartDashboard.putData("Swerve", swerve);
        // SmartDashboard.putData("Intake", intake);
        // SmartDashboard.putData("Hopper", hopper);
        // SmartDashboard.putData("Shooter", shooter);
        // SmartDashboard.putData("Climber", climber);
        // SmartDashboard.putData("LEDs", leds);

        // ==================== SIMULATION TEST COMMANDS ====================
        // These commands are for testing the 3D shooter simulation in AdvantageScope.
        // They appear as buttons in Shuffleboard/SmartDashboard.

        // Reset robot pose to a sensible position (~3m from Blue Hub, facing toward it)
        SmartDashboard.putData("Sim/Reset Pose", Commands.runOnce(() -> {
            swerve.resetOdometry(new Pose2d(1.6, 4.0, Rotation2d.fromDegrees(0)));
        }).withName("ResetPose"));

        // Shoot at mid-range RPS (25 RPS, good for ~3m distance)
        SmartDashboard.putData("Sim/Shoot Default", Commands.runOnce(() -> {
            double rps = 25.0;
            double exitVelocity = FuelProjectile.calculateExitVelocity(rps);
            FuelProjectile.launch(swerve.getPose(), swerve.getRobotVelocity(), exitVelocity);
            SmartDashboard.putNumber("Sim/LastShotRPS", rps);
            SmartDashboard.putNumber("Sim/LastExitVelocity", exitVelocity);
        }).withName("ShootDefault"));

        // Shoot using vision distance (auto-calculate RPS from distance-to-RPS table)
        SmartDashboard.putData("Sim/Shoot Vision", Commands.runOnce(() -> {
            double distance = visionSim.getDistanceToTarget();
            double rps = shooter.getRPSForDistance(distance);
            double exitVelocity = FuelProjectile.calculateExitVelocity(rps);
            FuelProjectile.launch(swerve.getPose(), swerve.getRobotVelocity(), exitVelocity);
            SmartDashboard.putNumber("Sim/LastShotDistance", distance);
            SmartDashboard.putNumber("Sim/LastShotRPS", rps);
        }).withName("ShootVision"));

        // Shoot with custom RPS (read from SmartDashboard number input)
        SmartDashboard.putNumber("Sim/CustomRPS", 25.0);
        SmartDashboard.putData("Sim/Shoot Custom RPS", Commands.runOnce(() -> {
            double rps = SmartDashboard.getNumber("Sim/CustomRPS", 25.0);
            double exitVelocity = FuelProjectile.calculateExitVelocity(rps);
            FuelProjectile.launch(swerve.getPose(), swerve.getRobotVelocity(), exitVelocity);
        }).withName("ShootCustomRPS"));

        // Shoot at specific distances (for tuning the distance-to-RPS table)
        SmartDashboard.putNumber("Sim/TestDistance", 3.0);
        SmartDashboard.putData("Sim/Shoot At Distance", Commands.runOnce(() -> {
            double distance = SmartDashboard.getNumber("Sim/TestDistance", 3.0);
            double rps = shooter.getRPSForDistance(distance);
            double exitVelocity = FuelProjectile.calculateExitVelocity(rps);
            FuelProjectile.launch(swerve.getPose(), swerve.getRobotVelocity(), exitVelocity);
            SmartDashboard.putNumber("Sim/LastShotRPS", rps);
            SmartDashboard.putNumber("Sim/LastExitVelocity", exitVelocity);
        }).withName("ShootAtDistance"));

        // Estimate required RPS for a distance (physics calculation, no shot)
        SmartDashboard.putData("Sim/Estimate RPS", Commands.runOnce(() -> {
            double distance = SmartDashboard.getNumber("Sim/TestDistance", 3.0);
            double estimatedRPS = FuelProjectile.estimateRequiredRPS(distance);
            SmartDashboard.putNumber("Sim/EstimatedRPS", estimatedRPS);
        }).withName("EstimateRPS"));

        // ==================== ALIGNMENT TUNING ====================
        // Tunable PID gains — change these live in Shuffleboard before pressing Auto Align.
        // P: rad/s per radian of error (3.0 → 180°/s at 60° error)
        // D: damping (opposes rotation to prevent overshoot)
        // MaxOmega: caps angular velocity in rad/s
        SmartDashboard.putNumber("Align/P", 3.0);
        SmartDashboard.putNumber("Align/D", 0.1);
        SmartDashboard.putNumber("Align/ToleranceDeg", 2.0);
        SmartDashboard.putNumber("Align/MaxOmega", 6.0);

        // Auto-align to hub center using a local PIDController with live-tunable gains.
        // Uses its own PID (not YAGSL heading PID) so tuning doesn't affect normal driving.
        // Debug values are published each cycle under "Align/" in Shuffleboard.
        SmartDashboard.putData("Sim/Auto Align", Commands.defer(() -> {
            Translation2d hubCenter = Constants.VisionConstants.getHubCenter(Utils.isOnRedAlliance());
            Pose2d robotPose = swerve.getPose();
            double targetRad = Math.atan2(
                hubCenter.getY() - robotPose.getY(),
                hubCenter.getX() - robotPose.getX());

            PIDController pid = new PIDController(
                SmartDashboard.getNumber("Align/P", 3.0),
                0,
                SmartDashboard.getNumber("Align/D", 0.1));
            pid.enableContinuousInput(-Math.PI, Math.PI);
            pid.setTolerance(Math.toRadians(SmartDashboard.getNumber("Align/ToleranceDeg", 2.0)));

            // Diagnostic: show EXACTLY what values were used for target calculation
            SmartDashboard.putNumber("Align/TargetDeg", Math.toDegrees(targetRad));
            SmartDashboard.putNumber("Align/RobotX", robotPose.getX());
            SmartDashboard.putNumber("Align/RobotY", robotPose.getY());
            SmartDashboard.putNumber("Align/RobotHeadingDeg", robotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Align/HubX", hubCenter.getX());
            SmartDashboard.putNumber("Align/HubY", hubCenter.getY());
            SmartDashboard.putBoolean("Align/IsRedAlliance", Utils.isOnRedAlliance());

            return Commands.run(() -> {
                // Live-update gains from Shuffleboard each cycle
                pid.setP(SmartDashboard.getNumber("Align/P", 3.0));
                pid.setD(SmartDashboard.getNumber("Align/D", 0.1));
                pid.setTolerance(Math.toRadians(SmartDashboard.getNumber("Align/ToleranceDeg", 2.0)));

                double currentRad = swerve.getPose().getRotation().getRadians();
                double output = pid.calculate(currentRad, targetRad);
                double maxOmega = SmartDashboard.getNumber("Align/MaxOmega", 6.0);
                output = MathUtil.clamp(output, -maxOmega, maxOmega);

                // Debug telemetry — watch these in Shuffleboard to diagnose alignment
                SmartDashboard.putNumber("Align/CurrentDeg", Math.toDegrees(currentRad));
                SmartDashboard.putNumber("Align/ErrorDeg", Math.toDegrees(pid.getError()));
                SmartDashboard.putNumber("Align/PID_Output", output);
                SmartDashboard.putNumber("Align/P_Contrib", pid.getP() * pid.getError());
                SmartDashboard.putBoolean("Align/AtTarget", pid.atSetpoint());

                swerve.drive(new ChassisSpeeds(0, 0, output));
            }, swerve).until(pid::atSetpoint)
              .finallyDo(() -> {
                  swerve.drive(new ChassisSpeeds(0, 0, 0));
                  SmartDashboard.putString("Sim/AlignStatus", "LOCKED");
              });
        }, java.util.Set.of(swerve)).withTimeout(10).withName("AutoAlign"));

        // Auto-align to hub center then shoot once.
        SmartDashboard.putData("Sim/Align and Shoot", Commands.defer(() -> {
            Translation2d hubCenter = Constants.VisionConstants.getHubCenter(Utils.isOnRedAlliance());
            double targetRad = Math.atan2(
                hubCenter.getY() - swerve.getPose().getY(),
                hubCenter.getX() - swerve.getPose().getX());

            PIDController pid = new PIDController(
                SmartDashboard.getNumber("Align/P", 3.0),
                0,
                SmartDashboard.getNumber("Align/D", 0.1));
            pid.enableContinuousInput(-Math.PI, Math.PI);
            pid.setTolerance(Math.toRadians(SmartDashboard.getNumber("Align/ToleranceDeg", 2.0)));

            return Commands.run(() -> {
                pid.setP(SmartDashboard.getNumber("Align/P", 3.0));
                pid.setD(SmartDashboard.getNumber("Align/D", 0.1));
                pid.setTolerance(Math.toRadians(SmartDashboard.getNumber("Align/ToleranceDeg", 2.0)));

                double currentRad = swerve.getPose().getRotation().getRadians();
                double output = pid.calculate(currentRad, targetRad);
                double maxOmega = SmartDashboard.getNumber("Align/MaxOmega", 6.0);
                output = MathUtil.clamp(output, -maxOmega, maxOmega);

                SmartDashboard.putNumber("Align/ErrorDeg", Math.toDegrees(pid.getError()));
                SmartDashboard.putNumber("Align/PID_Output", output);
                SmartDashboard.putBoolean("Align/AtTarget", pid.atSetpoint());

                swerve.drive(new ChassisSpeeds(0, 0, output));
            }, swerve).until(pid::atSetpoint)
              .finallyDo(() -> swerve.drive(new ChassisSpeeds(0, 0, 0)))
              .andThen(Commands.runOnce(() -> {
                  SmartDashboard.putString("Sim/AlignStatus", "SHOOTING");
                  double distance = swerve.getPose().getTranslation().getDistance(hubCenter);
                  double rps = shooter.getRPSForDistance(distance);
                  double exitVelocity = FuelProjectile.calculateExitVelocity(rps);
                  FuelProjectile.launch(swerve.getPose(), swerve.getRobotVelocity(), exitVelocity);
                  SmartDashboard.putNumber("Sim/LastShotDistance", distance);
                  SmartDashboard.putNumber("Sim/LastShotRPS", rps);
              }));
        }, java.util.Set.of(swerve)).withTimeout(10).withName("AlignAndShoot"));

        // ==================== ALIGNMENT AUTO-TUNING ====================
        // Uses AdambotsLib PIDAutoTuner (Ziegler-Nichols relay feedback) to find optimal P, I, D.
        // Workflow: Reset Pose → Tune Alignment → Apply Tuned Gains → Auto Align
        // IMPORTANT: Reset pose first (heading ≈ 0°) to avoid angle wrapping during oscillation.
        Command tuneCmd = PIDAutoTuner.create("HeadingAlign")
            .measureWith(() -> swerve.getPose().getRotation().getRadians())
            .controlWith(omega -> swerve.drive(new ChassisSpeeds(0, 0, omega)))
            .range(-Math.PI, Math.PI)
            .maxOutput(3.0)    // ±3.0 rad/s during oscillation (~170°/s)
            .timeout(15.0)
            .buildCommand();

        SmartDashboard.putData("Sim/Tune Alignment", Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Sim/TuneStatus", "TUNING..."), swerve),
            tuneCmd,
            Commands.runOnce(() -> {
                swerve.drive(new ChassisSpeeds(0, 0, 0));
                TuningResult result = PIDAutoTuner.getLastResult("HeadingAlign");
                if (result != null) {
                    SmartDashboard.putString("Sim/TuneStatus",
                        String.format("Done: P=%.4f I=%.4f D=%.4f", result.kP(), result.kI(), result.kD()));
                } else {
                    SmartDashboard.putString("Sim/TuneStatus", "Tuning failed — no result");
                }
            })
        ).finallyDo(() -> swerve.drive(new ChassisSpeeds(0, 0, 0)))
         .withName("TuneAlignment"));

        // Apply auto-tuned gains scaled to 50% (Z-N "no overshoot" rule).
        // Raw Z-N gains target ~25% overshoot; halving gives a conservative starting point.
        SmartDashboard.putData("Sim/Apply Tuned Gains", Commands.runOnce(() -> {
            TuningResult result = PIDAutoTuner.getLastResult("HeadingAlign");
            if (result != null) {
                double scaledP = result.kP() * 0.5;
                double scaledD = result.kD() * 0.5;
                SmartDashboard.putNumber("Align/P", scaledP);
                SmartDashboard.putNumber("Align/D", scaledD);
                SmartDashboard.putString("Sim/TuneStatus",
                    String.format("Applied (50%%): P=%.4f D=%.4f (raw: P=%.4f D=%.4f)",
                        scaledP, scaledD, result.kP(), result.kD()));
            } else {
                SmartDashboard.putString("Sim/TuneStatus", "No result — run Tune Alignment first");
            }
        }).withName("ApplyTunedGains"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Gets the swerve subsystem for simulation pose updates.
     *
     * @return The SwerveSubsystem instance
     */
    public SwerveSubsystem getSwerve() {
        return swerve;
    }

    /**
     * Gets the vision simulation subsystem.
     *
     * @return The VisionSimSubsystem instance
     */
    public VisionSimSubsystem getVisionSim() {
        return visionSim;
    }
}
