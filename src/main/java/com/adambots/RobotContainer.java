// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.IntakeCommands;
import com.adambots.commands.ShootCommands;
import com.adambots.commands.test.PIDTunerTestCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.VisionSimSubsystem;
import com.adambots.subsystems.test.TestTurretSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    /** Vision simulation subsystem for AprilTag detection in simulation */
    private final VisionSimSubsystem visionSim;

    /** Test turret subsystem for PID tuning experiments */
    private final TestTurretSubsystem testTurret;

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

        // Initialize test turret for PID tuning experiments
        testTurret = new TestTurretSubsystem(RobotMap.kTestTurretMotor);

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
     * Configure PhotonVision cameras for the swerve subsystem.
     *
     * <p>Uses VisionConfigBuilder from AdambotsLib to configure cameras:
     * <ul>
     *   <li>addCamera(name) - Camera name in PhotonVision</li>
     *   <li>position(x, y, z) - Position relative to robot center using Distance units</li>
     *   <li>rotation(roll, pitch, yaw) - Camera angle using Angle units</li>
     *   <li>purpose(CameraPurpose) - APRIL_TAGS or OBJECT_DETECTION</li>
     *   <li>singleTagStdDevs / multiTagStdDevs - Pose estimation trust levels</li>
     * </ul>
     *
     * TODO: Update camera names and positions for your robot
     */
    private void configureVision() {
        // TODO: Configure vision cameras using VisionConfigBuilder
        // import static edu.wpi.first.units.Units.*;
        // import com.adambots.lib.vision.config.*;
        //
        // VisionSystemConfig visionConfig = VisionConfigBuilder.create()
        //     .addCamera("front_camera")
        //         .position(Meters.of(0.3), Meters.of(0), Meters.of(0.5))
        //         .rotation(Degrees.of(0), Degrees.of(-20), Degrees.of(0))
        //         .purpose(VisionCameraConfig.CameraPurpose.APRIL_TAGS)
        //         .done()
        //     .addCamera("back_camera")
        //         .position(Meters.of(-0.3), Meters.of(0), Meters.of(0.5))
        //         .rotation(Degrees.of(0), Degrees.of(-20), Degrees.of(180))
        //         .purpose(VisionCameraConfig.CameraPurpose.APRIL_TAGS)
        //         .done()
        //     .ambiguityThreshold(0.2)  // Reject ambiguous poses
        //     .build();
        //
        // swerve.setupPhotonVision(visionConfig);
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
        // Get the joystick from Buttons
        CommandJoystick joystick = Buttons.getJoystick();

        // Swerve drive default command - field-oriented drive using joystick
        swerve.setDefaultCommand(
            swerve.driveCommand(
                () -> -Buttons.applyDeadzone(joystick.getY(), Constants.DriveConstants.kDeadzone),
                () -> -Buttons.applyDeadzone(joystick.getX(), Constants.DriveConstants.kDeadzone),
                () -> -Buttons.applyDeadzone(joystick.getTwist(), Constants.DriveConstants.kRotationDeadzone)
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

        // === PID Tuning Test Commands ===
        // These commands appear as buttons in SmartDashboard/Shuffleboard
        SmartDashboard.putData("PID Tuning/Tune Turret", PIDTunerTestCommands.tuneTurretPosition(testTurret));
        SmartDashboard.putData("PID Tuning/Test 90deg", PIDTunerTestCommands.testTunedPerformance(testTurret, 90));
        SmartDashboard.putData("PID Tuning/Test -90deg", PIDTunerTestCommands.testTunedPerformance(testTurret, -90));
        SmartDashboard.putData("PID Tuning/Test 45deg", PIDTunerTestCommands.testTunedPerformance(testTurret, 45));
        SmartDashboard.putData("PID Tuning/Center Turret", PIDTunerTestCommands.testTunedPerformance(testTurret, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // ==================== SECTION: SIMULATION GETTERS ====================

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

    /**
     * Gets the shooter subsystem for simulation shooting.
     *
     * @return The ShooterSubsystem instance
     */
    public ShooterSubsystem getShooter() {
        return shooter;
    }
}
