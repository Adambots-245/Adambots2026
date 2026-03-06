// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.ShootCommands;
import com.adambots.commands.TuningCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveConfig;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.InputCurve;
import com.adambots.lib.utils.Dash;
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
        intake = new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm);
        shooter = new ShooterSubsystem(RobotMap.shooterMotor2, RobotMap.shooterMotor1);
        turret = new TurretSubsystem(RobotMap.turretMotor);
        hopper = new HopperSubsystem(RobotMap.hopperMotor, RobotMap.uptakeMotor, RobotMap.hopperSensor);
        climber = new ClimberSubsystem(RobotMap.kClimberElevatorMotor, RobotMap.kClimberRatchetSolenoid,
                    RobotMap.kClimberRaisedLimit, RobotMap.kClimberLoweredLimit);
        leds = RobotMap.LEDS_ENABLED
                    ? new CANdleSubsystem(RobotMap.kCANdlePort)
               
                : null;

        // 3. Vision
        configureVision();

        // 4. LEDs
        configureLEDs();

        // 5. Default commands
        configureDefaultCommands();

        // 6. Button bindings
        configureButtonBindings();

        // 7. PathPlanner named commands
        configurePathPlannerCommands();

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
        leds.setDefaultCommand(leds.allianceColorCommand());
    }

    // ==================== DEFAULT COMMANDS ====================
    private void configureDefaultCommands() {
        // Swerve drive — negate translation on red so "forward" = away from red driver
        //
        // station
        boolean invertForRed = com.adambots.lib.utils.Utils.isOnRedAlliance();

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        Buttons.createForwardSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                        Buttons.createStrafeSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                        Buttons.createRotationSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true));

        // Turret auto-track: camera-only scan-and-track
        if (visionSubsystem != null) {
            turret.setDefaultCommand(turret.autoTrackCommand(
                        visionSubsystem::getHubCamAngle, visionSubsystem::isHubCamVisible));
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
                () -> intake.stopIntakeCommand()));

        // Trigger (1): Shoot (full sequence)
        Buttons.JoystickButton1.whileTrue(
                ShootCommands.shootAtDistanceCommand(
                        shooter, hopper, visionSubsystem::getHubDistance, intake, false));

        // Dash.addCommand("Shoot", ShootCommands.shootCommand(shooter, hopper));

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

        // Button 14: Lower intake but do not run
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

        // D-pad Left/Right: Manual turret adjust
        Buttons.XboxDPadW.whileTrue(
                    turret.scanCommand(1.0));
        Buttons.XboxDPadE.whileTrue(
                    turret.scanCommand(-1.0));

        // === Climber ===
        // D-pad Up: Extend elevator (hold to raise hook)
        Buttons.XboxDPadN.whileTrue(climber.extendCommand());
        // D-pad Down: Climb (hold to retract / pull robot up)
        Buttons.XboxDPadS.whileTrue(climber.climbCommand());
        // X: Lock climber (stop motor + engage ratchet)
        Buttons.XboxXButton.onTrue(climber.lockCommand());
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
                    // ShootCommands.shootCommand(shooter, hopper));
                    ShootCommands.shootAtDistanceCommand(
                                shooter, hopper, visionSubsystem::getHubDistance));
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
        DashboardSetup.configure(swerve, intake, shooter, turret, hopper, climber, visionSubsystem);
    }

    public void onTeleopInit(boolean noAutoRan) {
        turret.calibrateCommand().schedule();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
