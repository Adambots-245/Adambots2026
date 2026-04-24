// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static com.adambots.logging.LogUtil.DIAGNOSTIC;
import static com.adambots.logging.LogUtil.ESSENTIAL;
import static com.adambots.logging.LogUtil.log;

import java.io.File;

import com.adambots.Constants.VisionConstants;
import com.adambots.commands.DriveCommands;
// import com.adambots.commands.DriveCommands;  // uncomment when enabling the back-to-hub wiring below
import com.adambots.commands.LEDCommands;
import com.adambots.commands.ShootCommands;
import com.adambots.commands.TuningCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.CANdleSubsystem.AnimationTypes;
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
import com.adambots.utils.FieldGeometry;
import com.adambots.utils.HubActivation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
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

                HubActivation.nearEndTrigger(15)
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

                // Turret tracking: pose-lock tracker
                if (visionSubsystem != null) {
                        var xboxJog = (java.util.function.DoubleSupplier) () -> {
                                var xbox = Buttons.getXboxController();
                                return xbox != null ? xbox.getLeftX() : 0.0;
                        };
                        turret.setDefaultCommand(turret.poseTrackCommand(
                                        swerve::getPose,
                                        FieldGeometry::getHubCenter,
                                        () -> Math.toDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond),
                                        xboxJog));
                        // OPTION A: Pose tracker with translational lead compensation.
                        // Identical behavior to the above when
                        // Constants.TurretTrackingConstants.kShotLeadTimeSec is 0;
                        // once it's set non-zero (start around 0.15–0.20 s), the turret
                        // aims ahead of the hub to compensate for robot drift during
                        // fuel flight. Zero-safe: just set kShotLeadTimeSec = 0 to revert.
                        // turret.setDefaultCommand(turret.poseTrackCommand(
                        // swerve::getPose,
                        // FieldGeometry::getHubCenter,
                        // swerve::getFieldVelocity,
                        // () -> Math.toDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond),
                        // xboxJog));
                        // OPTION B: Pose tracker with TOF lead from vision distance
                        // (distance-dependent lead instead of constant).
                        // turret.setDefaultCommand(turret.poseTrackCommandTOF(
                        // swerve::getPose,
                        // FieldGeometry::getHubCenter,
                        // swerve::getFieldVelocity,
                        // () -> shooter.getEstimatedTOF(visionSubsystem.getHubDistance()),
                        // () -> Math.toDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond),
                        // xboxJog));
                } else {
                        turret.setDefaultCommand(turret.holdPositionCommand());
                }

                // Flywheel idle pre-spin — maintains low RPM between shots when enabled
                shooter.setDefaultCommand(shooter.idleCommand());
        }

        // ==================== BUTTON BINDINGS ====================
        private void configureButtonBindings() {
                // === Driver (Thrustmaster) ===
                // All the possible buttons are being mapped with unused ones registered as
                // Commands.none() to
                // avoid any conflicts in the future.

                // Trigger (1): Hold-to-shoot at vision distance (no timer)
                Buttons.JoystickButton1.whileTrue(
                                ShootCommands.holdShootAtDistanceCommand(
                                                shooter, hopper, turret, swerve, visionSubsystem::getHubDistance,
                                                visionSubsystem));
                // Button 2: Toggle bop
                Buttons.JoystickButton2.toggleOnTrue(intake.bopArmCommand());
                // Button 3: Drop arm and run intake
                Buttons.JoystickButton3.onTrue(
                                intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand()));
                // Button 4: Stop intake and raise arm
                Buttons.JoystickButton4.onTrue(
                                intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand().withTimeout(1)));

                // Button 5: Toggle auto-track on/off
                // Buttons.JoystickButton5.onTrue(turret.toggleAutoTrackCommand());

                // Alternative strategy — fixed turret, rotate chassis so its back faces the
                // hub.
                // Three wiring options (pick one, comment the auto-track toggle above):
                //
                // (1) Hold-to-aim, chassis stops translating while rotating:
                // Buttons.JoystickButton5.whileTrue(DriveCommands.backToHubCommand(swerve));
                //
                // (2) One-press auto-aim with safety timeout (ends at tolerance OR 1.5 s):
                // Buttons.JoystickButton5.onTrue(DriveCommands.backToHubCommand(swerve).withTimeout(1.5));
                //
                // (3) Hold-to-aim WITH driver translation passthrough — drive + auto-aim at
                // once.
                // Uses the same forward/strafe suppliers as the default drive command, scaled
                // to m/s via YAGSL's configured max chassis velocity:
                final double maxSpeed = swerve.getSwerveDrive().getMaximumChassisVelocity();
                final java.util.function.DoubleSupplier fwd = Buttons.createForwardSupplier(
                                Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true);
                final java.util.function.DoubleSupplier strf = Buttons.createStrafeSupplier(
                                Constants.DriveConstants.kDeadzone, InputCurve.CUBIC, true);
                // Buttons.JoystickButton5.whileTrue(DriveCommands.backToHubCommand(
                // swerve,
                // () -> fwd.getAsDouble() * maxSpeed *
                // Constants.DriveConstants.kTranslationScale,
                // () -> strf.getAsDouble() * maxSpeed *
                // Constants.DriveConstants.kTranslationScale,
                // 1.0 /* tolerance deg — settle detection prevents premature termination */));
                //
                // --- FRONT-facing-hub variants (mirror of the three above) ---
                // Use these for the forward-mounted shooter strategy: robot's FRONT faces the
                // hub. Same pose-trust guard and driver passthrough as the back variants.
                //
                // (1F) Hold-to-aim, chassis stops translating while rotating:
                // Buttons.JoystickButton5.whileTrue(DriveCommands.frontToHubCommand(swerve));
                //
                // (2F) One-press auto-aim with safety timeout (ends at tolerance OR 1.5 s):
                // Buttons.JoystickButton5.onTrue(DriveCommands.frontToHubCommand(swerve).withTimeout(1.5));
                //
                // (3F) Hold-to-aim WITH driver translation passthrough:
                Buttons.JoystickButton5.onTrue(DriveCommands.frontToHubCommand(
                                swerve,
                                () -> fwd.getAsDouble() * maxSpeed * Constants.DriveConstants.kTranslationScale,
                                () -> strf.getAsDouble() * maxSpeed * Constants.DriveConstants.kTranslationScale,
                                1.0 /*
                                     * tolerance deg — settle detection in DriveCommands prevents premature
                                     * termination
                                     */).withTimeout(1.5));

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
                // Button 13: Force pose reset from vision (only if cameras see 2+ tags)
                Buttons.JoystickButton13.onTrue(Commands.runOnce(() -> {
                        if (visionSubsystem != null && visionSubsystem.getHubVisibleTagCount() >= 2) {
                                swerve.resetOdometry(swerve.getPose());
                                System.out.println("[POSE RESET] Forced from vision — tags visible: "
                                                + visionSubsystem.getHubVisibleTagCount());
                        } else {
                                System.out.println("[POSE RESET] Skipped — not enough tags visible");
                        }
                }));
                // Button 14: None
                Buttons.JoystickButton14.onTrue(Commands.none());
                // Button 15: None
                Buttons.JoystickButton15.onTrue(Commands.none());

                // === Operator (Xbox Controller) ===

                // L Trigger: Bop (hold to bop, release to lower)
                Buttons.XboxLeftTriggerButton.whileTrue(intake.bopArmCommand());
                // R Trigger: Bop toggle (press to start, press again to stop and lower)
                Buttons.XboxRightTriggerButton.toggleOnTrue(intake.bopArmCommand());
                // Right Bumper: Intake up
                Buttons.XboxRightBumper.onTrue(intake.runRaiseIntakeArmCommand());
                // Left Bumper: Intake down
                Buttons.XboxLeftBumper.onTrue(intake.runLowerIntakeArmCommand());

                // Button A: Stop intake
                Buttons.XboxAButton.onTrue(intake.stopIntakeCommand());

                // B: Reverse Intake
                Buttons.XboxBButton.onTrue(intake.reverseIntakeCommand());

                // Y: Manual shoot — driver throttle controls flywheel speed, operator holds to
                // shoot
                Buttons.XboxYButton.whileTrue(
                                ShootCommands.manualShootCommand(shooter, hopper, Buttons.JoystickThrottle));

                // X: Toggle flywheel idle pre-spin
                Buttons.XboxXButton.onTrue(
                                Commands.runOnce(() -> shooter.setIdleEnabled(!shooter.isIdleEnabled())));

                // === D-pad: Turret manual control ===
                // Up = snap to forward (Motion Magic point-to-point)
                // Left/Right jog moved to Xbox left stick X (integrated into auto-track)
                Buttons.XboxDPadN.whileTrue(
                                turret.aimTurretCommand(() -> Constants.TurretConstants.kTurretForwardDegrees));

                // Start: Extend climber → lock when at top
                Buttons.XboxStartButton.onTrue(
                                climber.extendCommand()
                                                .until(climber::isAtRaisedLimit)
                                                .andThen(climber.lockCommand())
                                                .alongWith(leds.setAnimationCommand(AnimationTypes.Fire)));
                // Back: Retract climber → lock when at bottom
                Buttons.XboxBackButton.onTrue(
                                climber.retractCommand()
                                                .until(climber::isAtLoweredLimit)
                                                .andThen(climber.lockCommand())
                                                .alongWith(leds.setAnimationCommand(AnimationTypes.Fire)));
        }

        // ==================== PATHPLANNER ====================
        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("intake",
                                intake.runLowerIntakeArmCommand().andThen(
                                                intake.runIntakeCommand()));

                NamedCommands.registerCommand("shoot", ShootCommands.shootAtDistanceTimerWithBopCommand(
                                shooter, hopper, intake, turret, visionSubsystem::getHubDistance, visionSubsystem));

                NamedCommands.registerCommand("shootShortTimer", ShootCommands.shootAtDistanceTimerWithBopCommand(
                                shooter, hopper, intake, turret, visionSubsystem::getHubDistance, visionSubsystem)
                                .withTimeout(5.0));

                NamedCommands.registerCommand("shootLongTimer", ShootCommands.shootAtDistanceTimerWithBopCommand(
                                shooter, hopper, intake, turret, visionSubsystem::getHubDistance, visionSubsystem)
                                .withTimeout(5.0));

                NamedCommands.registerCommand("intakeUp", intake.runRaiseIntakeArmCommand());

                // Chassis-aim variants — pure rotation (no driver translation in auton).
                // Use at stop events between path segments; mid-path would conflict with the
                // path-follow command for SwerveSubsystem ownership. Settle detection in
                // DriveCommands terminates cleanly once the chassis is within ~1° and
                // not sweeping; 1.5s timeout is a safety backstop.
                NamedCommands.registerCommand("faceHub",
                                DriveCommands.frontToHubCommand(swerve).withTimeout(1.5));
                NamedCommands.registerCommand("backToHub",
                                DriveCommands.backToHubCommand(swerve).withTimeout(1.5));

                // Log PathPlanner trajectory target/current/active-path at ESSENTIAL.
                // These callbacks fire at PathPlanner's internal rate whenever a path-
                // following command is active. Without these, post-match auto debugging
                // is blind — we had this gap at MICMP1. Logger.recordOutput is thread-safe
                // per AdvantageKit docs, so off-main-thread callbacks are fine.
                PathPlannerLogging.setLogActivePathCallback(
                                poses -> log(ESSENTIAL, "PathPlanner/ActivePath", poses.toArray(new Pose2d[0])));
                PathPlannerLogging.setLogCurrentPoseCallback(pose -> log(ESSENTIAL, "PathPlanner/CurrentPose", pose));
                PathPlannerLogging.setLogTargetPoseCallback(pose -> log(ESSENTIAL, "PathPlanner/TargetPose", pose));
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
                // Dash.add registers suppliers that fire every Shuffleboard.update() tick
                // even with TUNING_ENABLED=false — gate them to avoid always-on NT chatter.
                // These two are duplicated in AdvantageKit ESSENTIAL logs ("Turret/Locked"
                // and "Vision/CamOnline"), so comp doesn't need them on Shuffleboard.
                if (Constants.TUNING_ENABLED) {
                        Dash.add("Auto-Track", () -> turret.isAutoTrackEnabled());
                        if (visionSubsystem != null) {
                                Dash.add("Camera Online", visionSubsystem::isCameraOnline);
                        }
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
                // If no auto ran (teleop-only practice), the pose starts at the
                // blue-side default (~1, 4). Mirror to red side if on red alliance
                // so the pose estimator converges quickly from nearby vision data.
                if (noAutoRan && com.adambots.lib.utils.Utils.isOnRedAlliance()
                                && swerve.getPose().getX() < 8.0) {
                        var current = swerve.getPose();
                        swerve.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(
                                        16.541 - current.getX(),
                                        current.getY(),
                                        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
                                                        180 - current.getRotation().getDegrees())));
                }
        }

        /**
         * Called from {@link Robot#disabledInit()}. Hook for subsystems to clean
         * up latched motor controller state before the next enable. Phoenix 6
         * keeps control requests latched in firmware across disable/enable, so
         * a subsystem that issued a MOTION_MAGIC target before a bad situation
         * will resume driving to that target the moment the robot is re-enabled
         * — unless the subsystem explicitly replaces the latched request here.
         *
         * <p>
         * Add new subsystem cleanup calls here as the need arises.
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

        /**
         * Logs swerve drive and steer motor applied output (duty cycle) for all 4
         * modules at {@link com.adambots.logging.LogUtil.Level#DIAGNOSTIC}.
         *
         * <p>
         * Called from {@link Robot#robotPeriodic()}. Swerve motors are managed by
         * YAGSL in the lib, so we log from outside.
         *
         * <p>
         * Note: this is <b>duty cycle (0–1)</b>, not current. Actual stator/supply
         * currents are logged to the CTRE {@code .hoot} file when {@code SignalLogger}
         * is enabled (see {@link Robot#robotInit()}) — use Tuner X → Log Extractor for
         * the real numbers. Demoted from ESSENTIAL because the duty cycle is a weak
         * proxy for current and PDH channel currents already cover the battery-side
         * view.
         */
        public void logSwerveCurrent() {
                if (!DIAGNOSTIC.enabled())
                        return;
                var modules = swerve.getSwerveDrive().getModules();
                for (int i = 0; i < modules.length && i < 4; i++) {
                        String name = modules[i].configuration.name;
                        log(DIAGNOSTIC, "Swerve/" + name + "/DriveOutput",
                                        modules[i].getDriveMotor().getAppliedOutput());
                        log(DIAGNOSTIC, "Swerve/" + name + "/SteerOutput",
                                        modules[i].getAngleMotor().getAppliedOutput());
                }
        }
}
