// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.LEDCommands;
import com.adambots.commands.ShootCommands;
import com.adambots.commands.TuningCommands;
import com.adambots.lib.Constants.LEDConstants;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveConfig;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.InputCurve;
import com.adambots.lib.utils.Dash;
import com.adambots.utils.HubActivation;
import com.adambots.lib.vision.VisionSystem;
import com.adambots.utils.DashboardSetup;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.VisionSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * RobotContainer — subsystems, commands, triggers, and button bindings.
 * Uses Inversion of Control (IoC) — hardware devices are created in RobotMap.
 */
@Logged
public class RobotContainer {

    // ==================== SUBSYSTEMS ====================
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final HopperSubsystem hopper;
    private final ClimberSubsystem climber;
    private final CANdleSubsystem leds;
    private VisionSubsystem visionSubsystem;
    private VisionSystem vision;

    private SendableChooser<Command> autoChooser;
    private Runnable tuningPeriodic;

    public RobotContainer() {
        // 1. Swerve config for PathPlanner
        SwerveConfig swerveConfig = new SwerveConfig()
                    .withTranslationPID(
                                Constants.DriveConstants.kAutoTranslationP,
                                Constants.DriveConstants.kAutoTranslationI,
                                Constants.DriveConstants.kAutoTranslationD)
                    .withRotationPID(
                                Constants.DriveConstants.kAutoRotationP,
                                Constants.DriveConstants.kAutoRotationI,
                                Constants.DriveConstants.kAutoRotationD)
                    .withEncoderAutoSync(true, 1.0)
                    .withTelemetryVerbosity(TelemetryVerbosity.POSE);
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), swerveConfig);

        // 2. Subsystems (IoC from RobotMap — dummy devices when disabled)
        intake = new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm, RobotMap.kIntakeArmEncoder);
        shooter = new ShooterSubsystem(RobotMap.shooterMotor2, RobotMap.shooterMotor1, swerve::getPose);
        turret = new TurretSubsystem(RobotMap.turretMotor, RobotMap.kTurretPotentiometer);
        hopper = new HopperSubsystem(RobotMap.hopperMotor, RobotMap.uptakeMotor, RobotMap.hopperSensor);
        climber = new ClimberSubsystem(RobotMap.kClimberElevatorMotor, RobotMap.kClimberRatchetSolenoid,
                    RobotMap.kClimberRaisedLimit, RobotMap.kClimberLoweredLimit);
        leds = RobotMap.LEDS_ENABLED
                    ? new CANdleSubsystem(RobotMap.kCANdlePort, Constants.kLEDStripLength, true)
                    : null;

        // 3. Vision
        configureVision();

        // 4. PathPlanner named commands — must be registered before buildAutoChooser()
        // which resolves them eagerly. Vision must be set up first (shoot commands use it).
        configurePathPlannerCommands();

        // 5. LEDs
        configureLEDs();

        // 6. Default commands
        configureDefaultCommands();

        // 7. Button bindings
        configureButtonBindings();

        // 8. Auto chooser
        configureAutoChooser();

        // 9. Dashboard
        configureDashboard();
    }

    // ==================== VISION ====================
    private void configureVision() {
        if (!RobotMap.BACK_CAMERAS_ENABLED && !RobotMap.SHOOTER_CAMERA_ENABLED)
            return;

        visionSubsystem = new VisionSubsystem(
                    swerve::getPose, swerve.getField(),
                    RobotMap.BACK_CAMERAS_ENABLED, RobotMap.SHOOTER_CAMERA_ENABLED);
        vision = visionSubsystem.getPhotonVision();
        swerve.setupVision(vision);
    }

    // ==================== LEDS ====================
    private void configureLEDs() {
        if (leds == null)
            return;

        // Enable hub activation test mode (dashboard controls for testing without FMS)
        HubActivation.initTestMode();

        // Default: green when active, alliance-color countdown when inactive, strobe at 5s
        leds.setDefaultCommand(LEDCommands.hubStateCommand(leds));

        // Flash green when hub becomes active
        HubActivation.ourHubActiveTrigger()
            .onTrue(LEDCommands.hubActivatedFlashCommand(leds));
    }

    // ==================== DEFAULT COMMANDS ====================
    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        Buttons.createForwardSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true),
                        Buttons.createStrafeSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true),
                        Buttons.createRotationSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true),
                        Constants.DriveConstants.kTranslationScale));

        // Turret auto-track: visible → track, not visible → search
        if (visionSubsystem != null) {
            turret.setDefaultCommand(turret.autoTrackCommand(
                        visionSubsystem::getHubCamAngle,
                        visionSubsystem::isHubCamVisible,
                        visionSubsystem::isHubCamFresh,
                        shooter::isInShootingZone));
        } else {
            turret.setDefaultCommand(turret.holdPositionCommand());
        }
    }

    // ==================== BUTTON BINDINGS ====================
    private void configureButtonBindings() {
        // === Driver (Thrustmaster) ===
        // Buttons.JoystickButton10.onTrue(Commands.runOnce(() ->
        //
        // swerve.zeroGyroWithAlliance()));
        Buttons.JoystickButton11.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        Buttons.XboxAButton.onTrue(Commands.runOnce(
                () -> intake.stopIntakeCommand()));

        // Trigger (1): Hold-to-shoot at vision distance (no timer)
        Buttons.JoystickButton1.whileTrue(
                ShootCommands.holdShootAtDistanceCommand(
                        shooter, hopper, turret, visionSubsystem::getHubDistance));

        // Button 2: Toggle bop
        Buttons.JoystickButton2.toggleOnTrue(intake.bopArmCommand());

        // Button 3: Toggle intake
        Buttons.JoystickButton3.onTrue(
                    // runLowerIntakeArmCommand is runOnce (sets Motion Magic target), so andThen
                    // fires immediately — roller spinning while arm deploys is
                //
                // intentional/harmless.
                    intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
        Buttons.JoystickButton4.onTrue(
                    // stopIntakeCommand is runOnce, so andThen fires before roller fully stops —
                    // arm raising while roller winds down is intentional/harmless.
                    intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand()));

        // Button 5: Toggle auto-track on/off
        Buttons.JoystickButton5.onTrue(turret.toggleAutoTrackCommand());

        // Button 6: Lob shot (hold) — intake + shoot at fixed RPS + feed hopper
        Buttons.JoystickButton6.whileTrue(
                    intake.runLowerIntakeArmCommand()
                                .andThen(ShootCommands.lobShotCommand(shooter, hopper, intake)));

        // Button 7: Bop and run intake
        Buttons.JoystickButton7.whileTrue(intake.bopArmAndRunCommand());

        // Button 12: Lower intake but do not run
        Buttons.JoystickButton12.onTrue(intake.runLowerIntakeArmCommand()); // TODO(vx-clutch): Drivers want this on the Xbox controller, however we have to many binds on that so we will have to discuss which to drop.

        // === Operator (Xbox Controller) ===
        // Right Trigger: Shoot with throttle-mapped RPS (vision fallback)
        Buttons.XboxRightTriggerButton.whileTrue(
                    ShootCommands.shootCommand(shooter, hopper, intake, false,
                                () -> shooter.throttleToRPS(Buttons.JoystickThrottle.getAsDouble())));

        // Left Trigger: Spin up flywheel (hold)
        Buttons.XboxLeftTriggerButton.whileTrue(
                    shooter.spinUpCommand());

        // Right Bumper: Feed hopper + uptake (manual)
        Buttons.XboxRightBumper.whileTrue(
                    hopper.feedCommand());

        // Left Bumper: Eject (reverse hopper + uptake, stop flywheel)
        Buttons.XboxLeftBumper.whileTrue(
                    ShootCommands.ejectCommand(shooter, hopper));

        // B: Stop all shooter systems
        Buttons.XboxBButton.onTrue(
                    ShootCommands.stopAllCommand(shooter, hopper));

        // Y: Hold turret at 90° while held — autoTrack resumes on release
        Buttons.XboxYButton.whileTrue(
                    turret.aimTurretCommand(() -> 90.0));

        // === D-pad: Turret manual control ===
        // Up = snap to forward (170°), Left/Right = incremental nudge
        // Diagonals included for POV hat wobble robustness
        double step = Constants.TurretConstants.kTurretManualStepDeg;
        Buttons.XboxDPadN.whileTrue(turret.aimTurretCommand(() -> Constants.TurretConstants.kTurretForwardDegrees));
        Buttons.XboxDPadE.whileTrue(turret.aimTurretCommand(() -> turret.getTurretAngleDegrees() + step));
        Buttons.XboxDPadW.whileTrue(turret.aimTurretCommand(() -> turret.getTurretAngleDegrees() - step));
        Buttons.XboxDPadNE.whileTrue(turret.aimTurretCommand(() -> turret.getTurretAngleDegrees() + step));
        Buttons.XboxDPadNW.whileTrue(turret.aimTurretCommand(() -> turret.getTurretAngleDegrees() - step));
        // X: Lock climber (stop motor + engage ratchet)
        Buttons.XboxXButton.onTrue(climber.lockCommand());

        // Back: One-press auto-extend — raise elevator to top, then lock
        Buttons.XboxBackButton.onTrue(
                    climber.extendCommand()
                                .until(climber::isAtRaisedLimit)
                                .andThen(climber.lockCommand()));

        // Start: One-press auto-climb — retract to bottom, then lock
        Buttons.XboxStartButton.onTrue(
                    climber.climbCommand()
                                .until(climber::isAtLoweredLimit)
                                .andThen(climber.lockCommand()));
    }

    // ==================== PATHPLANNER ====================
    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("intake",
                    intake.runLowerIntakeArmCommand().andThen(
                                intake.runIntakeCommand().withTimeout(Constants.IntakeConstants.kAutoIntakeTimeout)));
        NamedCommands.registerCommand("spinUp",
                    shooter.spinUpCommand()
                                .until(shooter.isAtSpeedTrigger())
                                .withTimeout(ShootCommands.kSpinUpTimeoutSeconds));
        NamedCommands.registerCommand("shoot",
                    ShootCommands.shootAtDistanceTimerCommand(
                                shooter, hopper, turret, visionSubsystem::getHubDistance));
        NamedCommands.registerCommand("shootWithBop",
                    ShootCommands.shootAtDistanceTimerWithBopCommand(
                                shooter, hopper, intake, turret, visionSubsystem::getHubDistance));
        NamedCommands.registerCommand("LowerIntakeArm", intake.runLowerIntakeArmCommand());
    }

    // ==================== AUTO CHOOSER ====================
    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();

        if (Constants.SWERVE_TAB) {
            // Swerve tuning commands (select from dashboard, run in auto mode)
            autoChooser.addOption("Tune Rotation PID", TuningCommands.tuneRotationPIDCommand(swerve));
            autoChooser.addOption("Tune Translation PID", TuningCommands.tuneTranslationPIDCommand(swerve));
            autoChooser.addOption("Estimate MOI", TuningCommands.estimateMOICommand(swerve));
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // ==================== DASHBOARD ====================
    private void configureDashboard() {
        Runnable tuning = DashboardSetup.configure(swerve, intake, shooter, turret, hopper, climber, visionSubsystem);
        tuningPeriodic = tuning != null ? tuning : () -> {};
    }

    /** Returns the tuning periodic callback. Safe to call every cycle (no-op when tuning disabled). */
    public Runnable getTuningPeriodic() {
        return tuningPeriodic;
    }

    public void onTeleopInit(boolean noAutoRan) {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
