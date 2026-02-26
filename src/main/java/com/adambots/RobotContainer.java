// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.ShootCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveConfig;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.InputCurve;
import com.adambots.lib.utils.Dash;
import com.adambots.lib.vision.VisionSystem;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.VisionSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * RobotContainer — subsystems, commands, triggers, and button bindings.
 * Uses Inversion of Control (IoC) — hardware devices are created in RobotMap.
 */
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ==================== CONSTRUCTOR ====================
    public RobotContainer() {
        // 1. Swerve
        SwerveConfig swerveConfig = new SwerveConfig()
            .withTranslationPID(
                Constants.DriveConstants.kAutoTranslationP,
                Constants.DriveConstants.kAutoTranslationI,
                Constants.DriveConstants.kAutoTranslationD)
            .withRotationPID(
                Constants.DriveConstants.kAutoRotationP,
                Constants.DriveConstants.kAutoRotationI,
                Constants.DriveConstants.kAutoRotationD);
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), swerveConfig);

        // 2. Subsystems (IoC from RobotMap)
        intake = RobotMap.INTAKE_ENABLED
            ? new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm) : null;
        shooter = RobotMap.SHOOTER_ENABLED
            ? new ShooterSubsystem(RobotMap.shooterLeftMotor, RobotMap.shooterRightMotor) : null;
        turret = RobotMap.TURRET_ENABLED
            ? new TurretSubsystem(RobotMap.turretMotor) : null;
        hopper = RobotMap.HOPPER_ENABLED
            ? new HopperSubsystem(RobotMap.hopperMotor, RobotMap.uptakeMotor, RobotMap.hopperSensor) : null;
        climber = RobotMap.CLIMBER_ENABLED
            ? new ClimberSubsystem(RobotMap.kClimberElevatorMotor, RobotMap.kClimberRatchetSolenoid) : null;
        leds = RobotMap.LEDS_ENABLED
            ? new CANdleSubsystem(RobotMap.kCANdlePort) : null;

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
        if (!RobotMap.BACK_CAMERAS_ENABLED && !RobotMap.SHOOTER_CAMERA_ENABLED) return;

        visionSubsystem = new VisionSubsystem(
            swerve::getPose, swerve.getField(),
            RobotMap.BACK_CAMERAS_ENABLED, RobotMap.SHOOTER_CAMERA_ENABLED);
        vision = visionSubsystem.getPhotonVision();
        swerve.setupVision(vision);
    }

    // ==================== LEDS ====================
    private void configureLEDs() {
        if (leds == null) return;
        leds.setDefaultCommand(leds.allianceColorCommand());
    }

    // ==================== DEFAULT COMMANDS ====================
    private void configureDefaultCommands() {
        // Swerve drive
        swerve.setDefaultCommand(
            swerve.driveCommand(
                Buttons.createForwardSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                Buttons.createStrafeSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC),
                Buttons.createRotationSupplier(Constants.DriveConstants.kDeadzone, InputCurve.CUBIC)
            )
        );

        // Turret auto-track: camera → pose fallback → oscillating scan
        // if (turret != null && visionSubsystem != null) {
        //     turret.setDefaultCommand(
        //         turret.autoTrackCommand(
        //             visionSubsystem::getHubCamAngle,
        //             visionSubsystem::isHubCamVisible,
        //             visionSubsystem::getTurretTargetAngle)
        //     );
        // }
    }

    // ==================== BUTTON BINDINGS ====================
    private void configureButtonBindings() {
        boolean hasShooterSystem = (shooter != null && hopper != null);

        // === Driver (Extreme 3D Pro) ===
        Buttons.JoystickButton2.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        if (hasShooterSystem) {
            // Trigger (1): Shoot (full sequence)
            Buttons.JoystickButton1.whileTrue(
                ShootCommands.shootCommand(shooter, hopper));

            // Thumb (2): already mapped to gyro reset above
            // Button 3: Toggle intake
            if (intake != null) {
                Buttons.JoystickButton3.onTrue(
                    // runLowerIntakeArmCommand is runOnce (sets Motion Magic target), so andThen
                    // fires immediately — roller spinning while arm deploys is intentional/harmless.
                    intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
                Buttons.JoystickButton4.onTrue(
                    // stopIntakeCommand is runOnce, so andThen fires before roller fully stops —
                    // arm raising while roller winds down is intentional/harmless.
                    intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand()));
            }
        }

        // === Operator (Xbox Controller) ===
        if (hasShooterSystem) {
            // Right Trigger: Shoot (full sequence)
            Buttons.XboxRightTriggerButton.whileTrue(
                ShootCommands.shootCommand(shooter, hopper));

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
        }

        if (turret != null) {
            // A: Smart scan for hub (position-controlled oscillating sweep)
            Buttons.XboxAButton.whileTrue(
                turret.scanForHubCommand());

            // Y: Hold turret at 0° while held — autoTrack resumes on release
            Buttons.XboxYButton.whileTrue(
                turret.aimTurretCommand(() -> 0.0));

            // D-pad Left/Right: Manual turret adjust
            Buttons.XboxDPadW.whileTrue(
                turret.scanCommand(Constants.TurretConstants.kTurretManualSpeed));
            Buttons.XboxDPadE.whileTrue(
                turret.scanCommand(-Constants.TurretConstants.kTurretManualSpeed));
        }
    }

    // ==================== PATHPLANNER ====================
    private void configurePathPlannerCommands() {
        if (intake != null) {
            NamedCommands.registerCommand("intake",
                intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
        }
        if (shooter != null) {
            NamedCommands.registerCommand("spinUp", shooter.spinUpCommand());
        }
        if (shooter != null && hopper != null) {
            NamedCommands.registerCommand("shoot",
                ShootCommands.shootCommand(shooter, hopper));
        }
    }

    // ==================== AUTO CHOOSER ====================
    private void configureAutoChooser() {
        autoChooser.setDefaultOption("None", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // ==================== DASHBOARD ====================
    private void configureDashboard() {
        if (shooter == null && turret == null) return;

        Dash.useTab("Shooter Tuning");
        int[] pos = {0, 0};
        int cols = 6;

        // Flywheel PID tunables
        if (shooter != null) {
            shooter.setupFlywheelTunables(pos, cols);
        }

        // Turret PID tunables
        if (turret != null) {
            if (pos[0] != 0) { pos[0] = 0; pos[1]++; }
            turret.setupTurretTunables(pos, cols);
        }

        // Live telemetry row
        if (pos[0] != 0) { pos[0] = 0; pos[1]++; }
        int telemetryRow = pos[1];

        if (visionSubsystem != null) {
            Dash.add("Vision Distance (m)", visionSubsystem::getHubDistance, 0, telemetryRow);
        }
        if (shooter != null) {
            Dash.add("Target RPS", shooter::getTargetRPS, 1, telemetryRow);
            Dash.add("Left RPS", shooter::getLeftRPS, 2, telemetryRow);
            Dash.add("Right RPS", shooter::getRightRPS, 3, telemetryRow);
            Dash.add("At Speed", shooter::isAtSpeed, 4, telemetryRow);
        }
        if (turret != null) {
            Dash.add("Turret Angle (deg)", turret::getTurretAngleDegrees, 5, telemetryRow);
        }
        if (visionSubsystem != null) {
            Dash.add("Hub Visible", visionSubsystem::isHubVisible, 6, telemetryRow);
        }

        Dash.useDefaultTab();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
