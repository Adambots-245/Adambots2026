// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static edu.wpi.first.units.Units.Centimeters;

import java.io.File;

import com.adambots.commands.ShootCommands;
import com.adambots.commands.SystemCheckCommand;
import com.adambots.commands.TuningCommands;
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
import com.pathplanner.lib.auto.AutoBuilder;
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

        // // Turret auto-track: camera → pose fallback → oscillating scan
        // if (visionSubsystem != null) {
        //     turret.setDefaultCommand(turret.autoTrackCommand(
        //         visionSubsystem::getHubCamAngle,
        //         visionSubsystem::isHubCamVisible,
        //         visionSubsystem::getTurretTargetAngle));
        // } else {
        //     turret.setDefaultCommand(turret.holdPositionCommand());
        // }
    }

    // ==================== BUTTON BINDINGS ====================
    private void configureButtonBindings() {
        // === Driver (Extreme 3D Pro) ===
        Buttons.JoystickButton2.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        // Trigger (1): Shoot (full sequence)
        Buttons.JoystickButton1.whileTrue(
            ShootCommands.shootCommand(shooter, hopper));
        
        Dash.addCommand("Shoot", ShootCommands.shootCommand(shooter, hopper));

        // Button 3: Toggle intake
        Buttons.JoystickButton3.onTrue(
            // runLowerIntakeArmCommand is runOnce (sets Motion Magic target), so andThen
            // fires immediately — roller spinning while arm deploys is intentional/harmless.
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

        // Y: Hold turret at 0° while held — autoTrack resumes on release
        Buttons.XboxYButton.whileTrue(
            turret.aimTurretCommand(() -> 0.0));

        // D-pad Left/Right: Manual turret adjust
        Buttons.XboxDPadW.whileTrue(
            turret.scanCommand(Constants.TurretConstants.kTurretManualSpeed));
        Buttons.XboxDPadE.whileTrue(
            turret.scanCommand(-Constants.TurretConstants.kTurretManualSpeed));

        // Left Stick Up/Down: Manual turret adjust
        Buttons.XboxLeftStickUp.whileTrue(
            turret.scanCommand(Constants.TurretConstants.kTurretManualSpeed));
        Buttons.XboxLeftStickDown.whileTrue(
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
            intake.runLowerIntakeArmCommand().andThen(
                intake.runIntakeCommand().withTimeout(Constants.IntakeConstants.kAutoIntakeTimeout)));
        NamedCommands.registerCommand("spinUp",
            shooter.spinUpCommand()
                .until(shooter.isAtSpeedTrigger())
                .withTimeout(ShootCommands.kSpinUpTimeoutSeconds));
        NamedCommands.registerCommand("shoot",
            ShootCommands.shootCommand(shooter, hopper));
    }

    // ==================== AUTO CHOOSER ====================
    private void configureAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();

        if (Constants.TUNING_ENABLED) {
            // Swerve tuning commands (select from dashboard, run in auto mode)
            autoChooser.addOption("Tune Rotation PID", TuningCommands.tuneRotationPIDCommand(swerve));
            autoChooser.addOption("Tune Translation PID", TuningCommands.tuneTranslationPIDCommand(swerve));
            autoChooser.addOption("Estimate MOI", TuningCommands.estimateMOICommand(swerve));
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // ==================== DASHBOARD ====================
    private void configureDashboard() {
        if (Constants.TUNING_ENABLED) {
            // Shooter Tuning tab — 10 cols × 5 rows visible
            // Row 0: Flywheel PID (5)          | Shoot button at col 8
            // Row 1: Dist 1-5 | RPS 1-5        (interpolation table)
            // Row 2: Lob RPS | Turret PID (5)
            // Row 3: Live telemetry (9 items)
            // Row 4: Exercise commands
            Dash.useTab("Shooter Tuning");
            int[] pos = {0, 0};
            int cols = Constants.kShuffleboardCols;

            // Row 0: Flywheel PID tunables (cols 0-4)
            shooter.setupFlywheelTunables(pos, cols);

            // Row 2 (continued): Turret PID tunables after Lob Shot
            turret.setupTurretTunables(pos, cols);

            // Row 3: Live telemetry
            if (pos[0] != 0) { pos[0] = 0; pos[1]++; }
            int telemetryRow = pos[1];

            int tc = 0;
            if (visionSubsystem != null) {
                Dash.add("Vision Dist (m)", visionSubsystem::getHubDistance, tc++, telemetryRow);
            }
            Dash.add("Target RPS", shooter::getTargetRPS, tc++, telemetryRow);
            Dash.add("Left RPS", shooter::getLeftRPS, tc++, telemetryRow);
            Dash.add("Right RPS", shooter::getRightRPS, tc++, telemetryRow);
            Dash.add("At Speed", shooter::isAtSpeed, tc++, telemetryRow);
            Dash.add("Turret Angle", turret::getTurretAngleDegrees, tc++, telemetryRow);
            Dash.add("Calibrated", turret::isCalibrated, tc++, telemetryRow);
            if (visionSubsystem != null) {
                Dash.add("Hub Visible", visionSubsystem::isHubVisible, tc++, telemetryRow);
                Dash.add("Alliance", visionSubsystem::getAllianceColor, tc++, telemetryRow);
            }

            // Row 4: Exercise commands for tuning workflow
            int cmdRow = telemetryRow + 1;
            int cc = 0;
            if (visionSubsystem != null) {
                Dash.addCommand("Shoot", ShootCommands.shootAtDistanceCommand(
                    shooter, hopper, visionSubsystem::getHubDistance), cc++, cmdRow);
            } else {
                Dash.addCommand("Shoot", ShootCommands.shootCommand(shooter, hopper), cc++, cmdRow);
            }
            Dash.addCommand("Spin Up", shooter.spinUpCommand(), cc++, cmdRow);
            Dash.addCommand("Stop Flywheel", shooter.stopFlywheelCommand(), cc++, cmdRow);
            Dash.addCommand("Feed Hopper", hopper.feedCommand(), cc++, cmdRow);
            Dash.addCommand("Stop All", ShootCommands.stopAllCommand(shooter, hopper), cc++, cmdRow);
            Dash.addCommand("Lob Shot",
                intake.runLowerIntakeArmCommand()
                    .andThen(ShootCommands.lobShotCommand(shooter, hopper, intake))
                    .withName("Lob Shot"), cc++, cmdRow);
            Dash.addCommand("Eject", ShootCommands.ejectCommand(shooter, hopper), cc++, cmdRow);
            Dash.addCommand("Calibrate Turret", turret.calibrateCommand(), cc++, cmdRow);
            Dash.addCommand("Turret to 0", turret.aimTurretCommand(() -> 0.0), cc++, cmdRow);

            Dash.useDefaultTab();

            // Commands tab — mirrors button bindings for controllerless testing
            Dash.useTab("Commands");
            int col = 0, row = 0;

            // Driver commands
            Dash.addCommand("Shoot", ShootCommands.shootAtDistanceCommand(shooter, hopper, visionSubsystem::getHubDistance), col++, row);
            Dash.addCommand("Zero Gyro", Commands.runOnce(() -> swerve.zeroGyro()).withName("Zero Gyro"), col++, row);
            Dash.addCommand("Lower + Intake",
                intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand())
                    .withName("Lower + Intake"), col++, row);
            Dash.addCommand("Stop + Raise",
                intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand())
                    .withName("Stop + Raise"), col++, row);
            Dash.addCommand("Toggle AutoTrack", turret.toggleAutoTrackCommand(), col++, row);
            Dash.addCommand("Lob Shot",
                intake.runLowerIntakeArmCommand()
                    .andThen(ShootCommands.lobShotCommand(shooter, hopper, intake))
                    .withName("Lob Shot"), col++, row);

            // Operator commands
            col = 0; row++;
            Dash.addCommand("Spin Up", shooter.spinUpCommand(), col++, row);
            Dash.addCommand("Feed Hopper", hopper.feedCommand(), col++, row);
            Dash.addCommand("Eject", ShootCommands.ejectCommand(shooter, hopper), col++, row);
            Dash.addCommand("Stop All", ShootCommands.stopAllCommand(shooter, hopper), col++, row);
            Dash.addCommand("Scan Turret", turret.scanCommand(Constants.TurretConstants.kTurretManualSpeed), col++, row);
            Dash.addCommand("Turret to 0", turret.aimTurretCommand(() -> 0.0), col++, row);
            if (visionSubsystem != null) {
                Dash.addCommand("Track Hub",
                    turret.trackHubCommand(visionSubsystem::getHubAngle, visionSubsystem::isHubVisible)
                        .withName("Track Hub"), col++, row);
            }

            // Climber commands
            col = 0; row++;
            Dash.addCommand("Extend Climber", climber.extendCommand(), col++, row);
            Dash.addCommand("Climb", climber.climbCommand(), col++, row);
            Dash.addCommand("Lock Climber", climber.lockCommand(), col++, row);

            // Utility
            Dash.addCommand("Calibrate Turret", turret.calibrateCommand(), col++, row);
            Dash.addCommand("Stop Flywheel", shooter.stopFlywheelCommand(), col++, row);
            Dash.addCommand("Reverse Hopper", hopper.reverseCommand(), col++, row);

            Dash.useDefaultTab();
        }

        // System Check tab — passive health + exercise buttons for pit crew
        Dash.useTab("System Check");

        // Passive health indicators (auto-updating, dynamic row count)
        SystemCheckCommand sysCheck = new SystemCheckCommand(swerve,
            RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm,
            RobotMap.shooterLeftMotor, RobotMap.shooterRightMotor,
            RobotMap.turretMotor,
            RobotMap.hopperMotor, RobotMap.uptakeMotor,
            RobotMap.kClimberElevatorMotor,
            RobotMap.kClimberRatchetSolenoid);

        // Exercise commands — flow dynamically after health indicators
        int col = 0, row = sysCheck.getRowCount();
        int sysCols = Constants.kShuffleboardCols;

        // Swerve
        Dash.addCommand("Steer Modules",
            Commands.runOnce(() -> swerve.lock(), swerve).withName("Steer Modules"), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Center Modules",
            swerve.centerModulesCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Drive Forward",
            swerve.driveForwardDistanceCommand(0.5, 0.3), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Run Intake",
            intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand())
                .withName("Run Intake"), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Stop Intake",
            intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand())
                .withName("Stop Intake"), col, row);
        if (++col >= sysCols) { col = 0; row++; }

        // Shooter + turret
        Dash.addCommand("Spin Flywheel", shooter.spinUpCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Stop Flywheel", shooter.stopFlywheelCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Move Turret", turret.scanCommand(Constants.TurretConstants.kTurretManualSpeed), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Turret to 0", turret.aimTurretCommand(() -> 0.0), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Calibrate Turret", turret.calibrateCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }

        // Hopper + climber
        Dash.addCommand("Feed Hopper", hopper.feedCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Stop Hopper", hopper.stopCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Extend Climber", climber.extendCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Lock Climber", climber.lockCommand(), col, row);
        if (++col >= sysCols) { col = 0; row++; }

        // Live sensor
        Dash.add("Hopper Sensor (cm)",
            () -> RobotMap.hopperSensor.getDistance().in(Centimeters), col, row);
        Dash.useDefaultTab();
    }

    public void onTeleopInit() {
        turret.calibrateCommand().schedule();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
