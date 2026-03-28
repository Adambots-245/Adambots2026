// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.util.HashMap;
import java.util.Map;

import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.ControllerType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after
 * creating this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer container;

    // For tracking command execution times
    private final Map<Command, Double> commandStartTimes = new HashMap<>();

    /**
     * Constructor - AdvantageKit Logger MUST be configured here, before LoggedRobot initialization.
     */
    public Robot() {
        // Configure AdvantageKit Logger FIRST (required before LoggedRobot parent init)
        Logger.recordMetadata("ProjectName", "Adambots2026");

        if (!isReal()) {
            // In simulation only — publish to NT for live AdvantageScope viewing.
            Logger.addDataReceiver(new NT4Publisher());
        }
        // Log to USB stick on real robot for post-match analysis in AdvantageScope
        Logger.addDataReceiver(new WPILOGWriter());

        Logger.start();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // 1. WPILib DataLogManager — logs DS data, joystick inputs to USB for post-match review
        DataLogManager.start();

        // CTRE SignalLogger — logs all TalonFX signals (current, voltage, velocity) to .hoot file
        // Runs on CANivore processor, no roboRIO CPU impact. Uncomment to enable.
        // com.ctre.phoenix6.SignalLogger.start();

        // 2. Initialize buttons with driver joystick (Extreme 3D Pro) and operator Xbox controller
        Buttons.init(
            RobotMap.kDriverJoystickPort,
            RobotMap.kOperatorXboxPort,
            ControllerType.EXTREME_3D_PRO,
            ControllerType.XBOX
        );

        // 3. Create RobotContainer (creates all subsystems)
        container = new RobotContainer();

        // 4. CommandScheduler timing hooks disabled — high overhead from per-command logging
        // setupCommandSchedulerHooks();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Time the entire scheduler run (includes command execute + subsystem periodic)
        double schedulerStart = Timer.getFPGATimestamp();

        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        container.getTuningPeriodic().run();

        double schedulerMs = (Timer.getFPGATimestamp() - schedulerStart) * 1000.0;
        Logger.recordOutput("Timing/CommandSchedulerTotal", schedulerMs);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = container.getAutonomousCommand();

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        container.onTeleopInit(autonomousCommand == null);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        org.ironmaple.simulation.SimulatedArena.getInstance();
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId(
            edu.wpi.first.hal.AllianceStationID.Red1);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
        System.out.println("[SIM] Auto-enabled as Red1");

        // Print hub tag positions for debugging
        try {
            var layout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
                edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
            int[] redHub = {2, 3, 4, 5, 8, 9, 10, 11};
            int[] blueHub = {18, 19, 20, 21, 24, 25, 26, 27};
            for (int id : redHub) {
                var pose = layout.getTagPose(id);
                if (pose.isPresent()) {
                    var t = pose.get().getTranslation();
                    System.out.printf("[FIELD] Red hub tag %d: x=%.2f y=%.2f z=%.2f%n", id, t.getX(), t.getY(), t.getZ());
                } else {
                    System.out.printf("[FIELD] Red hub tag %d: NOT FOUND in field layout%n", id);
                }
            }
        } catch (Exception e) {
            System.out.println("[FIELD] Could not load field layout: " + e.getMessage());
        }

        // Start robot ~3m from Red hub, facing toward it (0° = positive X)
        // Red hub center at x≈11.9, y≈4.0
        container.getSwerve().resetOdometry(
            new edu.wpi.first.math.geometry.Pose2d(
                13.5, 4.0,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        var robotPose = container.getSwerve().getPose();
        double turretAngle = container.getTurretAngle();
        if (container.getVision() != null) {
            container.getVision().simulationPeriodic(robotPose, turretAngle);
        }
        org.ironmaple.simulation.SimulatedArena.getInstance().simulationPeriodic();

        // Publish field-relative camera pose for AdvantageScope Camera Override
        double robotYaw = robotPose.getRotation().getRadians();
        double camYaw = robotYaw + Math.toRadians(
            Constants.VisionConstants.kShooterCameraTurretOffset - turretAngle);
        double camX = robotPose.getX()
            + Constants.VisionConstants.kShooterCameraX * Math.cos(camYaw)
            - Constants.VisionConstants.kShooterCameraY * Math.sin(camYaw);
        double camY = robotPose.getY()
            + Constants.VisionConstants.kShooterCameraX * Math.sin(camYaw)
            + Constants.VisionConstants.kShooterCameraY * Math.cos(camYaw);
        org.littletonrobotics.junction.Logger.recordOutput("CameraPose",
            new edu.wpi.first.math.geometry.Pose3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    camX, camY, Constants.VisionConstants.kShooterCameraZ),
                new edu.wpi.first.math.geometry.Rotation3d(
                    0, Math.toRadians(-Constants.VisionConstants.kShooterCameraPitch), camYaw)));
    }

    /**
     * Sets up CommandScheduler hooks to log command timing for troubleshooting overruns.
     * Logs:
     * - Commands/Running/{name}: Boolean indicating if command is active
     * - Commands/Duration/{name}: How long the command ran (in milliseconds)
     */
    private void setupCommandSchedulerHooks() {
        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.onCommandInitialize(command -> {
            commandStartTimes.put(command, Timer.getFPGATimestamp());
            Logger.recordOutput("Commands/Running/" + command.getName(), true);
        });

        scheduler.onCommandFinish(command -> {
            Double startTime = commandStartTimes.remove(command);
            if (startTime != null) {
                double durationMs = (Timer.getFPGATimestamp() - startTime) * 1000.0;
                Logger.recordOutput("Commands/Duration/" + command.getName(), durationMs);
            }
            Logger.recordOutput("Commands/Running/" + command.getName(), false);
        });

        scheduler.onCommandInterrupt(command -> {
            commandStartTimes.remove(command);
            Logger.recordOutput("Commands/Running/" + command.getName(), false);
        });
    }
}
