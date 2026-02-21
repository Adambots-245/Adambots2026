// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.IntakeCommands;
import com.adambots.commands.ShootCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.InputCurve;
import com.adambots.lib.utils.Dash;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionSystem;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;


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

    /** Intake subsystem for acquiring game pieces (null if disabled in RobotMap) */
    private final IntakeSubsystem intake;

    /** Hopper subsystem for storing and staging game pieces (null if disabled in RobotMap) */
    private final HopperSubsystem hopper;

    /** Shooter subsystem for launching game pieces (null if disabled in RobotMap) */
    private final ShooterSubsystem shooter;

    /** Climber subsystem for end-game climbing (null if disabled in RobotMap) */
    private final ClimberSubsystem climber;

    /** LED subsystem using CANdle for robot status indication (null if disabled in RobotMap) */
    private final CANdleSubsystem leds;

    /** PhotonVision system for AprilTag detection and pose estimation */
    private VisionSystem vision;

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
        // Subsystems are only created when their enable flag is true in RobotMap
        intake = RobotMap.INTAKE_ENABLED
            ? new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeSensor) : null;
        hopper = RobotMap.HOPPER_ENABLED
            ? new HopperSubsystem(RobotMap.kHopperCarouselMotor, RobotMap.kHopperUptakeMotor, RobotMap.kHopperSensor) : null;
        shooter = RobotMap.SHOOTER_ENABLED
            ? new ShooterSubsystem(RobotMap.kShooterLeftMotor, RobotMap.kShooterRightMotor, RobotMap.kShooterTurretMotor) : null;
        climber = RobotMap.CLIMBER_ENABLED
            ? new ClimberSubsystem(RobotMap.kClimberLeftMotor, RobotMap.kClimberRightMotor,
                                          RobotMap.kClimberLeftLimit, RobotMap.kClimberRightLimit) : null;
        leds = RobotMap.LEDS_ENABLED
            ? new CANdleSubsystem(RobotMap.kCANdlePort) : null;

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
        if (leds == null) return;

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
        Dash.add("Z", ()->Buttons.getJoystick().getZ());

        // Swerve drive default command - field-oriented drive using joystick
        swerve.setDefaultCommand(
            swerve.driveCommand(
                Buttons.createForwardSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                Buttons.createStrafeSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                Buttons.createRotationSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC)
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
        // Reset gyro heading with joystick button 2
        Buttons.JoystickButton2.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

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
        if (shooter != null) {
            NamedCommands.registerCommand("spinUp", shooter.spinUpCommand());
        }

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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
