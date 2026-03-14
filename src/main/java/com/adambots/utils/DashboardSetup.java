package com.adambots.utils;

import static edu.wpi.first.units.Units.Centimeters;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.RobotMap;
import com.adambots.commands.ShootCommands;
import com.adambots.commands.SystemCheckCommand;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Dash;
import com.adambots.subsystems.ClimberSubsystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveModule;

/**
 * Configures all Shuffleboard tabs and dashboard widgets.
 * Extracted from RobotContainer to keep it focused on subsystem wiring.
 */
public final class DashboardSetup {
    private DashboardSetup() {}

    /**
     * Configures all dashboard tabs and returns a tuning poll Runnable.
     * @return a Runnable to call each cycle for tuning reads, or null if TUNING_ENABLED is false
     */
    public static Runnable configure(
            SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            ClimberSubsystem climber,
            VisionSubsystem visionSubsystem) {

        TuningManager tuningManager = null;

        if (Constants.TUNING_ENABLED) {
            tuningManager = new TuningManager(shooter, turret, intake, hopper, visionSubsystem);
        }

        if (Constants.SHOOTER_TAB)  configureShooterTuningTab(shooter, turret, hopper, intake, visionSubsystem, tuningManager);
        if (Constants.CLIMBER_TAB)  configureClimberTab(climber);
        if (Constants.SWERVE_TAB)   configureSwerveTab(swerve);
        if (Constants.COMMANDS_TAB) configureCommandsTab(swerve, intake, shooter, turret, hopper, climber, visionSubsystem);
        if (Constants.INTAKE_TAB && tuningManager != null)  tuningManager.setupIntakeTunables();
        if (Constants.HOPPER_TAB && tuningManager != null)  tuningManager.setupHopperTunables();

        configureSystemCheckTab(swerve, intake, shooter, turret, hopper, climber);

        return tuningManager != null ? tuningManager::periodic : null;
    }

    // ==================== SHOOTER TUNING TAB ====================
    private static void configureShooterTuningTab(
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            IntakeSubsystem intake,
            VisionSubsystem visionSubsystem,
            TuningManager tuningManager) {

        Dash.useTab("Shooter");
        int[] pos = {0, 0};
        int cols = Constants.kShuffleboardCols;

        // Tunable entries (delegated to TuningManager)
        if (tuningManager != null) {
            tuningManager.setupShooterTunables(pos, cols);
            tuningManager.setupTurretTunables(pos, cols);
        }

        // Live telemetry row
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
        if (visionSubsystem != null) {
            Dash.add("Hub Visible", visionSubsystem::isHubVisible, tc++, telemetryRow);
            Dash.add("Alliance", visionSubsystem::getAllianceColor, tc++, telemetryRow);
        }

        // Row 4: Exercise commands for tuning workflow
        int cmdRow = telemetryRow + 1;
        int cc = 0;
        if (visionSubsystem != null) {
            Dash.addCommand("Shoot (Dist)", ShootCommands.shootAtDistanceTimerCommand(
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
        Dash.addCommand("Turret Left",
            turret.scanCommand(1.0), cc++, cmdRow);
        Dash.addCommand("Turret Right",
            turret.scanCommand(-1.0), cc++, cmdRow);
        Dash.addCommand("Turret Forward",
            turret.aimTurretCommand(TurretConstants.kTurretForwardDegrees), cc++, cmdRow);

        // Vision tunables row (after commands)
        if (visionSubsystem != null) {
            int visionRow = cmdRow + 1;
            int vc = 0;

            if (tuningManager != null) {
                tuningManager.setupVisionTunables(vc, visionRow);
                vc += 2;
            }

            Dash.addCommand("Log Vision", visionSubsystem.logVisionCommand(), vc++, visionRow);

            // Raw pre-filter values for AdvantageScope analysis
            int rawRow = visionRow + 1;
            int rc = 0;
            Dash.add("Raw Cam Dist", visionSubsystem::getRawCamDist, rc++, rawRow);
            Dash.add("Raw Cam Angle", visionSubsystem::getRawCamAngle, rc++, rawRow);
            Dash.add("Raw Pose Dist", visionSubsystem::getRawPoseDist, rc++, rawRow);
            Dash.add("Raw Pose Angle", visionSubsystem::getRawPoseAngle, rc++, rawRow);
        }

        Dash.useDefaultTab();
    }

    // ==================== SWERVE DIAGNOSTICS TAB ====================
    private static void configureSwerveTab(SwerveSubsystem swerve) {
        Dash.useTab("Swerve");
        SwerveModule[] modules = swerve.getSwerveDrive().getModules();
        String[] names = {"FL", "FR", "BL", "BR"};

        // Row 0: Chassis state
        Dash.add("Pose X", () -> swerve.getPose().getX(), 0, 0);
        Dash.add("Pose Y", () -> swerve.getPose().getY(), 1, 0);
        Dash.add("Heading", () -> swerve.getHeading().getDegrees(), 2, 0);
        Dash.add("Field Vel", () -> {
            var v = swerve.getFieldVelocity();
            return Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
        }, 3, 0);

        // Rows 1-5: Per-module data
        for (int i = 0; i < modules.length; i++) {
            final int idx = i;
            Dash.add(names[i] + " Abs°", () -> modules[idx].getAbsolutePosition(), i, 1);
            Dash.add(names[i] + " Rel°", () -> modules[idx].getState().angle.getDegrees(), i, 2);
            Dash.add(names[i] + " Err°", () -> modules[idx].getAbsolutePosition() - modules[idx].getState().angle.getDegrees(), i, 3);
            Dash.add(names[i] + " m/s", () -> modules[idx].getState().speedMetersPerSecond, i, 4);
            Dash.add(names[i] + " OK", () -> !modules[idx].getAbsoluteEncoderReadIssue(), i, 5);
        }

        // Row 6: Commands
        Dash.addCommand("Sync Encoders",
            Commands.runOnce(() -> swerve.getSwerveDrive().synchronizeModuleEncoders(), swerve)
                .withName("Sync Encoders"), 0, 6);
        Dash.addCommand("Center Modules", swerve.centerModulesCommand(), 1, 6);
        Dash.addCommand("Lock Modules",
            Commands.runOnce(() -> swerve.lock(), swerve).withName("Lock"), 2, 6);

        Dash.useDefaultTab();
    }

    // ==================== COMMANDS TAB ====================
    private static void configureCommandsTab(
            SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            ClimberSubsystem climber,
            VisionSubsystem visionSubsystem) {

        Dash.useTab("Commands");
        int col = 0, row = 0;

        // Driver commands
        if (visionSubsystem != null) {
            Dash.addCommand("Shoot (Dist)", ShootCommands.shootAtDistanceTimerCommand(shooter, hopper, visionSubsystem::getHubDistance), col++, row);
        } else {
            Dash.addCommand("Shoot", ShootCommands.shootCommand(shooter, hopper), col++, row);
        }
        Dash.addCommand("Zero Gyro", Commands.runOnce(() -> swerve.zeroGyro()).withName("Zero Gyro"), col++, row);
        Dash.addCommand("Lower + Intake",
            intake.runLowerIntakeArmCommand().andThen(intake.runIntakeCommand())
                .withName("Lower + Intake"), col++, row);
        Dash.addCommand("Stop + Raise",
            intake.stopIntakeCommand().andThen(intake.runRaiseIntakeArmCommand())
                .withName("Stop + Raise"), col++, row);
        // Dash.addCommand("Toggle AutoTrack", turret.toggleAutoTrackCommand(), col++, row);
        Dash.addCommand("Lob Shot",
            intake.runLowerIntakeArmCommand()
                .andThen(ShootCommands.lobShotCommand(shooter, hopper, intake))
                .withName("Lob Shot"), col++, row);
        if (visionSubsystem != null) {
            Dash.addCommand("Shoot + Bop", Commands.parallel(
                ShootCommands.shootAtDistanceTimerCommand(shooter, hopper, visionSubsystem::getHubDistance),
                intake.bopArmCommand()
            ).withName("Shoot + Bop"), col++, row);
        }

        // Operator commands
        col = 0; row++;
        Dash.addCommand("Spin Up", shooter.spinUpCommand(), col++, row);
        Dash.addCommand("Feed Hopper", hopper.feedCommand(), col++, row);
        Dash.addCommand("Eject", ShootCommands.ejectCommand(shooter, hopper), col++, row);
        Dash.addCommand("Stop All", ShootCommands.stopAllCommand(shooter, hopper), col++, row);
        if (visionSubsystem != null) {
            Dash.addCommand("Auto Track",
                turret.autoTrackCommand(
                    visionSubsystem::getHubCamAngle, visionSubsystem::isHubCamVisible,
                    visionSubsystem::getHubPoseAngle, visionSubsystem::isHubPoseVisible,
                    () -> swerve.getHeading().getRadians(),
                    () -> swerve.getFieldVelocity().vxMetersPerSecond,
                    () -> swerve.getFieldVelocity().vyMetersPerSecond,
                    visionSubsystem::getHubDistance)
                    .withName("Auto Track"), col++, row);
        }

        // Climber commands
        col = 0; row++;
        Dash.addCommand("Extend Climber", climber.extendCommand(), col++, row);
        Dash.addCommand("Climb", climber.climbCommand(), col++, row);
        Dash.addCommand("Lower", climber.lowerCommand(), col++, row);
        Dash.addCommand("Lock Climber", climber.lockCommand(), col++, row);

        // Utility
        Dash.addCommand("Stop Flywheel", shooter.stopFlywheelCommand(), col++, row);
        Dash.addCommand("Reverse Hopper", hopper.reverseCommand(), col++, row);
        if (visionSubsystem != null) {
            Dash.addCommand("Log Vision", visionSubsystem.logVisionCommand(), col++, row);
        }

        // Diagnostic: stepped wizard that prints turret tracking results to console
        if (visionSubsystem != null) {
            col = 0; row++;
            Dash.add("Diag Step", turret::getDiagInstruction, col++, row);
            Dash.addCommand("Turret Diag",
                turret.turretDiagnosticCommand(
                    visionSubsystem::getHubCamAngle, visionSubsystem::isHubCamVisible,
                    visionSubsystem::getHubPoseAngle, visionSubsystem::isHubPoseVisible,
                    () -> swerve.getHeading().getRadians(),
                    () -> swerve.getFieldVelocity().vxMetersPerSecond,
                    () -> swerve.getFieldVelocity().vyMetersPerSecond,
                    visionSubsystem::getHubDistance,
                    () -> {
                        var p = swerve.getPose();
                        return String.format("(%.2f, %.2f)", p.getX(), p.getY());
                    })
                    .withName("Turret Diagnostic"), col++, row);
        }

        Dash.useDefaultTab();
    }

    // ==================== CLIMBER TAB ====================
    private static void configureClimberTab(ClimberSubsystem climber) {
        Dash.useTab("Climber");
        int col = 0, row = 0;

        // Row 0: Live sensor readouts
        Dash.add("Raised Limit", climber::isAtRaisedLimit, col++, row);
        Dash.add("Lowered Limit", climber::isAtLoweredLimit, col++, row);
        Dash.add("Ratchet Engaged", climber::isRatchetEngaged, col++, row);
        Dash.add("Motor Position", () -> RobotMap.kClimberElevatorMotor.getPosition(), col++, row);

        // Row 1: Exercise commands
        col = 0; row++;
        Dash.addCommand("Extend", climber.extendCommand(), col++, row);
        Dash.addCommand("Retract", climber.retractCommand(), col++, row);
        Dash.addCommand("Climb", climber.climbCommand(), col++, row);
        Dash.addCommand("Lower", climber.lowerCommand(), col++, row);
        Dash.addCommand("Lock", climber.lockCommand(), col++, row);
        Dash.addCommand("Engage Ratchet", climber.engageRatchetCommand(), col++, row);
        Dash.addCommand("Release Ratchet", climber.releaseRatchetCommand(), col++, row);

        Dash.useDefaultTab();
    }

    // ==================== SYSTEM CHECK TAB ====================
    private static void configureSystemCheckTab(
            SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            HopperSubsystem hopper,
            ClimberSubsystem climber) {

        Dash.useTab("System Check");

        // Passive health indicators (auto-updating, dynamic row count)
        SystemCheckCommand sysCheck = new SystemCheckCommand(swerve,
            RobotMap.kIntakeMotor, RobotMap.kIntakeMotorArm,
            RobotMap.shooterMotor2, RobotMap.shooterMotor1,
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
        Dash.addCommand("Move Turret", turret.scanCommand(1.0), col, row);
        if (++col >= sysCols) { col = 0; row++; }
        Dash.addCommand("Turret to 0", turret.aimTurretCommand(() -> 0.0), col, row);
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
}
