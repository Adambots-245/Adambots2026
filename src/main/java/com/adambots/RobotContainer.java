// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static edu.wpi.first.units.Units.Centimeters;

import java.io.File;

import com.adambots.commands.SelfTestCommand;
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
                Constants.DriveConstants.kAutoRotationD);
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), swerveConfig);

        // 2. Subsystems (IoC from RobotMap — dummy devices when disabled)
        intake = new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm);
        shooter = new ShooterSubsystem(RobotMap.shooterLeftMotor, RobotMap.shooterRightMotor);
        turret = new TurretSubsystem(RobotMap.turretMotor);
        hopper = new HopperSubsystem(RobotMap.hopperMotor, RobotMap.uptakeMotor, RobotMap.hopperSensor);
        climber = new ClimberSubsystem(RobotMap.kClimberElevatorMotor, RobotMap.kClimberRatchetSolenoid);
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
        // === Driver (Extreme 3D Pro) ===
        Buttons.JoystickButton2.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        // Trigger (1): Shoot (full sequence)
        Buttons.JoystickButton1.whileTrue(
            ShootCommands.shootCommand(shooter, hopper));

        // Button 3: Toggle intake
        Buttons.JoystickButton3.onTrue(
            // runLowerIntakeArmCommand is runOnce (sets Motion Magic target), so andThen
            // fires immediately — roller spinning while arm deploys is intentional/harmless.
            intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
        Buttons.JoystickButton4.onTrue(
            // stopIntakeCommand is runOnce, so andThen fires before roller fully stops —
            // arm raising while roller winds down is intentional/harmless.
            intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand()));

        // === Operator (Xbox Controller) ===
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
            intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
        NamedCommands.registerCommand("spinUp", shooter.spinUpCommand());
        NamedCommands.registerCommand("shoot",
            ShootCommands.shootCommand(shooter, hopper));
    }

    // ==================== AUTO CHOOSER ====================
    private void configureAutoChooser() {
        autoChooser.setDefaultOption("None", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // ==================== DASHBOARD ====================
    private void configureDashboard() {
        Dash.useTab("Shooter Tuning");
        int[] pos = {0, 0};
        int cols = 6;

        // Flywheel PID tunables
        shooter.setupFlywheelTunables(pos, cols);

        // Turret PID tunables
        if (pos[0] != 0) { pos[0] = 0; pos[1]++; }
        turret.setupTurretTunables(pos, cols);

        // Live telemetry row
        if (pos[0] != 0) { pos[0] = 0; pos[1]++; }
        int telemetryRow = pos[1];

        if (visionSubsystem != null) {
            Dash.add("Vision Distance (m)", visionSubsystem::getHubDistance, 0, telemetryRow);
        }
        Dash.add("Target RPS", shooter::getTargetRPS, 1, telemetryRow);
        Dash.add("Left RPS", shooter::getLeftRPS, 2, telemetryRow);
        Dash.add("Right RPS", shooter::getRightRPS, 3, telemetryRow);
        Dash.add("At Speed", shooter::isAtSpeed, 4, telemetryRow);
        Dash.add("Turret Angle (deg)", turret::getTurretAngleDegrees, 5, telemetryRow);
        if (visionSubsystem != null) {
            Dash.add("Hub Visible", visionSubsystem::isHubVisible, 6, telemetryRow);
        }

        Dash.useDefaultTab();

        // Self-Test tab — one-shot device check for pit crew
        // Layout: button row 0, device boxes start row 1, health info below
        Dash.useTab("Self-Test");
        SelfTestCommand selfTest = new SelfTestCommand(
            Dash.getCurrentTab(), 1,
            RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm,
            RobotMap.shooterLeftMotor, RobotMap.shooterRightMotor,
            RobotMap.turretMotor,
            RobotMap.hopperMotor, RobotMap.uptakeMotor,
            RobotMap.kClimberElevatorMotor,
            RobotMap.kClimberRatchetSolenoid);
        Dash.addCommand("Run Self-Test", selfTest, 0, 0);
        // Live sensor — wave hand over CANRange to verify it's working
        Dash.add("Hopper Sensor (cm)",
            () -> RobotMap.hopperSensor.getDistance().in(Centimeters), 0, 2);
        Dash.useDefaultTab();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
