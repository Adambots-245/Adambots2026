// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.Constants.ShooterConstants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.commands.LEDCommands;
import com.adambots.commands.ShootCommands;
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
import com.adambots.utils.DashboardSetup;
import com.adambots.utils.HubActivation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
                                // .withEncoderAutoSync(true, 1.0)
                                .withTelemetryVerbosity(TelemetryVerbosity.POSE);
                swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), swerveConfig);

                // 2. Subsystems (IoC from RobotMap — dummy devices when disabled)
                intake = new IntakeSubsystem(RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm);
                shooter = new ShooterSubsystem(RobotMap.shooterMotor2, RobotMap.shooterMotor1, swerve::getPose);
                turret = new TurretSubsystem(RobotMap.turretMotor, RobotMap.kTurretPotentiometer);
                hopper = new HopperSubsystem(RobotMap.hopperMotor, RobotMap.uptakeMotor);
                climber = new ClimberSubsystem(RobotMap.kClimberElevatorMotor, RobotMap.kClimberRatchetSolenoid,
                                RobotMap.kClimberRaisedLimit, RobotMap.kClimberLoweredLimit);
                leds = RobotMap.LEDS_ENABLED
                                ? new CANdleSubsystem(RobotMap.kCANdlePort, Constants.kLEDStripLength, true)
                                : null;

                // 3. Vision
                configureVision();

                // 4. PathPlanner named commands — must be registered before buildAutoChooser()
                // which resolves them eagerly. Vision must be set up first (shoot commands use
                // it).
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
                                RobotMap.BACK_CAMERAS_ENABLED, RobotMap.SHOOTER_CAMERA_ENABLED,
                                RobotMap.FRONT_CAMERA_ENABLED);
                vision = visionSubsystem.getPhotonVision();
                swerve.setupVision(vision);
                visionSubsystem.setTurretAngleSupplier(turret::getTurretAngleDegrees);

                // Scale vision std devs by robot speed — trust vision less when driving fast
                vision.setStdDevScaler(() -> {
                        double speed = Math.hypot(
                                swerve.getRobotVelocity().vxMetersPerSecond,
                                swerve.getRobotVelocity().vyMetersPerSecond);
                        return 1.0 + speed * VisionConstants.kVisionSpeedScaling;
                });
        }

        // ==================== LEDS ====================
        private void configureLEDs() {
                if (leds == null)
                        return;

                // Enable hub activation test mode (dashboard controls for testing without FMS)
                HubActivation.initTestMode();

                // Default: green when active, alliance-color countdown when inactive, strobe at
                // 5s
                leds.setDefaultCommand(LEDCommands.hubStateCommand(leds));

                // Flash green when hub becomes active
                HubActivation.ourHubActiveTrigger()
                                .onTrue(LEDCommands.hubActivatedFlashCommand(leds));

                // Rumble operator controller when 5 seconds until shift change
                HubActivation.shiftChangeSoonTrigger(5.0)
                                .onTrue(Commands.runOnce(() -> Buttons.rumbleOperator(1000, 1.0)));
        }

        // ==================== DEFAULT COMMANDS ====================
        private void configureDefaultCommands() {
                swerve.setDefaultCommand(
                                swerve.driveCommand(
                                                Buttons.createForwardSupplier(Constants.DriveConstants.kDeadzone,
                                                                InputCurve.CUBIC, true),
                                                Buttons.createStrafeSupplier(Constants.DriveConstants.kDeadzone,
                                                                InputCurve.CUBIC, true),
                                                Buttons.createRotationSupplier(Constants.DriveConstants.kDeadzone,
                                                                InputCurve.CUBIC, true),
                                                Constants.DriveConstants.kTranslationScale));

                // Turret auto-track: visible → track, not visible → search
                if (visionSubsystem != null) {
                        turret.setDefaultCommand(turret.autoTrackCommand(
                                        visionSubsystem::getHubAngle,
                                        visionSubsystem::isHubVisible,
                                        visionSubsystem::isTrackingDataFresh,
                                        shooter::isInShootingZone,
                                        () -> Math.toDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond)));
                } else {
                        turret.setDefaultCommand(turret.holdPositionCommand());
                }

                // Flywheel idle pre-spin — maintains low RPM between shots when enabled
                shooter.setDefaultCommand(shooter.idleCommand());
        }

        // ==================== BUTTON BINDINGS ====================
        private void configureButtonBindings() {
                // === Driver (Thrustmaster) ===
                // All the possible buttons are being mapped with unused ones registered as Commands.none() to 
                // avoid any conflicts in the future.

                // Trigger (1): Hold-to-shoot at vision distance with chassis shake (no timer)
                Buttons.JoystickButton1.whileTrue(
                                ShootCommands.holdShootAtDistanceCommand(
                                                shooter, hopper, turret, swerve, visionSubsystem::getHubDistance, visionSubsystem));
                // Button 2: Toggle bop
                Buttons.JoystickButton2.toggleOnTrue(intake.bopArmCommand());
                // Button 3: Drop arm and run intake
                Buttons.JoystickButton3.onTrue(
                                intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
                // Button 4: Stop intake and raise arm
                Buttons.JoystickButton4.onTrue(
                                intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand().withTimeout(1)));

                // Button 5: Toggle auto-track on/off
                Buttons.JoystickButton5.onTrue(turret.toggleAutoTrackCommand());
                // Button 6: Lower Intake Arm withour running rollers
                Buttons.JoystickButton6.whileTrue(
                                intake.runLowerIntakeArmCommand());
                // Button 7: Reverse a jam in the shooter/hopper
                Buttons.JoystickButton7.onTrue(ShootCommands.ejectCommand(shooter, hopper).withTimeout(0.5));
                // Button 8: None
                Buttons.JoystickButton8.onTrue(Commands.none());
                // Button 9: Bop and run intake
                Buttons.JoystickButton9.whileTrue(intake.bopArmAndRunCommand());
                // Button 10: Lob
                Buttons.JoystickButton10.whileTrue(ShootCommands.lobShotCommand(shooter, hopper, intake));
                // Button 11: Zero Gyro
                Buttons.JoystickButton11.onTrue(Commands.runOnce(() -> swerve.zeroGyro()));
                // Button 12: Lower intake but do not run
                Buttons.JoystickButton12.onTrue(intake.runLowerIntakeArmCommand()); 
                // Button 13: None
                Buttons.JoystickButton13.onTrue(Commands.none());
                // Button 14: None
                Buttons.JoystickButton14.onTrue(Commands.none());
                // Button 15: None
                Buttons.JoystickButton15.onTrue(Commands.none());

                // === Operator (Xbox Controller) ===

                // R-L Triggers: Bop
                Buttons.XboxLeftTriggerButton.whileTrue(intake.bopArmCommand());
                Buttons.XboxRightTriggerButton.whileTrue(intake.bopArmCommand());
                // Right Bumper: Intake up
                Buttons.XboxRightBumper.onTrue(intake.runRaiseIntakeArmCommand());
                // Left Bumper: Intake down
                Buttons.XboxLeftBumper.onTrue(intake.runLowerIntakeArmCommand());

                // Button A: Stop intake
                Buttons.XboxAButton.onTrue(intake.stopIntakeCommand());

                // B: Reverse Intake
                Buttons.XboxBButton.onTrue(intake.reverseIntakeCommand());

                // Y: Manual shoot — driver throttle controls flywheel speed, operator holds to shoot
                Buttons.XboxYButton.whileTrue(
                                ShootCommands.manualShootCommand(shooter, hopper, Buttons.JoystickThrottle));

                // X: Toggle flywheel idle pre-spin
                Buttons.XboxXButton.onTrue(
                        Commands.runOnce(() -> shooter.setIdleEnabled(!shooter.isIdleEnabled())));

                // === D-pad: Turret manual control ===
                // Up = snap to forward (Motion Magic point-to-point)
                // Left/Right = hold-to-jog (percent output, NOT Motion Magic —
                // Diagonals included for POV hat wobble robustness
                Buttons.XboxDPadN.whileTrue(
                                turret.aimTurretCommand(() -> Constants.TurretConstants.kTurretForwardDegrees));
                Buttons.XboxDPadE.whileTrue(turret.scanCommand(+1.0));
                Buttons.XboxDPadW.whileTrue(turret.scanCommand(-1.0));
                Buttons.XboxDPadNE.whileTrue(turret.scanCommand(+1.0));
                Buttons.XboxDPadNW.whileTrue(turret.scanCommand(-1.0));

                // Start: None
                Buttons.XboxStartButton.onTrue(Commands.none());
                                                // climber.extendCommand()
                                                // .until(climber::isAtRaisedLimit)
                                                // .andThen(climber.lockCommand()));
                // Back: None
                Buttons.XboxBackButton.onTrue(Commands.none());
                                                // climber.retractCommand()
                                                // .until(climber::isAtLoweredLimit)
                                                // .andThen(climber.lockCommand()));    

        }

        // ==================== PATHPLANNER ====================
        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("intake",
                                intake.runLowerIntakeArmCommand().andThen(
                                                intake.runIntakeCommand()));

                 NamedCommands.registerCommand("intakeTimer",
                                intake.runLowerIntakeArmCommand().withTimeout(1));
                                                
                NamedCommands.registerCommand("spinUp",
                                shooter.spinUpCommand()
                                                .until(shooter.isAtSpeedTrigger())
                                                .withTimeout(ShootCommands.kSpinUpTimeoutSeconds));
                NamedCommands.registerCommand("shoot", Commands.sequence(
                                turret.aimTurretCommand(() -> TurretConstants.kTurretForwardDegrees).withTimeout(1.5),
                                ShootCommands.shootAtDistanceTimerCommand(
                                                shooter, hopper, turret, visionSubsystem::getHubDistance, visionSubsystem)));
                NamedCommands.registerCommand("legacyShoot",
                                ShootCommands.shootAtDistanceTimerCommand(
                                                shooter, hopper, turret, visionSubsystem::getHubDistance, visionSubsystem));
                NamedCommands.registerCommand("shootNoPoint", Commands.race(
                                Commands.waitSeconds(3),
                                ShootCommands.shootAtDistanceTimerCommand(shooter, hopper, turret,
                                                visionSubsystem::getHubDistance, visionSubsystem)));
                NamedCommands.registerCommand("shootWithBop",
                                ShootCommands.shootAtDistanceTimerWithBopCommand(
                                                shooter, hopper, intake, turret, visionSubsystem::getHubDistance, visionSubsystem));
                NamedCommands.registerCommand("LowerIntakeArm", intake.runLowerIntakeArmCommand());
                NamedCommands.registerCommand("intakeLob",
                                ShootCommands.autonLobCommand(shooter, turret, hopper, intake));
                NamedCommands.registerCommand("intakeUp", intake.runRaiseIntakeArmCommand());
                NamedCommands.registerCommand("aimForward",
                                turret.aimTurretCommand(() -> TurretConstants.kTurretForwardDegrees).withTimeout(1.5));
                NamedCommands.registerCommand("waitForHub",
                                visionSubsystem.waitForHubCommand().withTimeout(2.0));
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
                Runnable tuning = DashboardSetup.configure(swerve, intake, shooter, turret, hopper, climber,
                                visionSubsystem);
                tuningPeriodic = tuning != null ? tuning : () -> {
                };
                Dash.add("Auto-Track", () -> turret.isAutoTrackEnabled());
                if (visionSubsystem != null) {
                        Dash.add("Camera Online", visionSubsystem::isCameraOnline);
                }
        }

        /**
         * Returns the tuning periodic callback. Safe to call every cycle (no-op when
         * tuning disabled).
         */
        public Runnable getTuningPeriodic() {
                return tuningPeriodic;
        }

        public void onTeleopInit(boolean noAutoRan) {
        }

        /**
         * Called from {@link Robot#disabledInit()}. Hook for subsystems to clean
         * up latched motor controller state before the next enable. Phoenix 6
         * keeps control requests latched in firmware across disable/enable, so
         * a subsystem that issued a MOTION_MAGIC target before a bad situation
         * will resume driving to that target the moment the robot is re-enabled
         * — unless the subsystem explicitly replaces the latched request here.
         *
         * <p>Add new subsystem cleanup calls here as the need arises.
         */
        public void onDisabledInit() {
                if (intake != null) {
                        intake.onDisable();
                }
                // Future: turret.onDisable(), shooter.onDisable(), etc.
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
