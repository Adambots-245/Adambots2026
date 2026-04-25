// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import static com.adambots.logging.LogUtil.DIAGNOSTIC;
import static com.adambots.logging.LogUtil.ESSENTIAL;
import static com.adambots.logging.LogUtil.log;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import com.adambots.lib.utils.Buttons;
import com.adambots.lib.utils.Buttons.ControllerType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
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
     *
     * <p>Receiver configuration is driven by {@link Constants#MODE}:
     * <ul>
     *   <li>{@code REAL} → WPILOGWriter (defaults to {@code /U/logs} if USB stick mounted,
     *       else {@code /home/lvuser/logs}) + NT4Publisher for live AdvantageScope.</li>
     *   <li>{@code SIM} → NT4Publisher only (no disk writes in sim).</li>
     *   <li>{@code REPLAY} → reads a saved log, writes a new {@code _sim} log alongside it;
     *       no NT or hardware contact.</li>
     * </ul>
     */
    public Robot() {
        // Configure AdvantageKit Logger FIRST (required before LoggedRobot parent init)
        Logger.recordMetadata("ProjectName", "Adambots2026");
        Logger.recordMetadata("LogLevel", Constants.LOG_LEVEL.name());
        Logger.recordMetadata("Mode", Constants.MODE.name());
        Logger.recordMetadata("BatteryId", String.valueOf(Constants.BATTERY_ID));

        switch (Constants.MODE) {
            case REAL:
                // WPILOGWriter default path picks /U (USB stick) if mounted, else /home/lvuser/logs
                Logger.addDataReceiver(new WPILOGWriter());
                // NT4Publisher has no measurable CPU/bandwidth impact per MICMP1 analysis
                // (LogPeriodicMS mean was 2.3ms); keep on for live dashboards.
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                // Tie robot clock to log playback so commands/triggers fire in order.
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Log the configured battery ID once at boot as an ESSENTIAL signal so
        // it appears on the AdvantageScope timeline in addition to the metadata
        // header. Lets match-analysis scripts correlate sag/brownout events to
        // a specific physical battery (see MICMP1 Q19/Q58 forensics).
        log(ESSENTIAL, "System/BatteryId", Constants.BATTERY_ID);

        // 1. WPILib DataLogManager — logs DS data, joystick inputs to USB for post-match review
        DataLogManager.start();

        // CTRE SignalLogger — logs all TalonFX signals (stator/supply current, voltage,
        // velocity, position, temperature) at 1 kHz to .hoot file. Runs on CANivore
        // processor with zero roboRIO CPU impact. Writes to /media/sda1/ctre-logs/ if a
        // USB stick is present, else /home/lvuser/logs. See Tuner X → Log Extractor.
        //
        // Only enabled in REAL mode (avoids filling sim workstation disk). If no USB stick
        // is mounted, disable to avoid filling RIO flash — we leave enablement to the
        // team's deploy workflow via this single gate.
        if (Constants.MODE == Constants.Mode.REAL && hasUsbStorage()) {
            com.ctre.phoenix6.SignalLogger.start();
        }

        // 2. Initialize buttons with driver joystick (Extreme 3D Pro) and operator Xbox controller
        Buttons.init(
            RobotMap.kDriverJoystickPort,
            RobotMap.kOperatorXboxPort,
            ControllerType.EXTREME_3D_PRO,
            ControllerType.XBOX
        );

        // 3. Create RobotContainer (creates all subsystems)
        container = new RobotContainer();

        // 4. CommandScheduler timing hooks — Running/* at ESSENTIAL, Duration/* at DIAGNOSTIC.
        //    The callbacks themselves only fire on command transitions (not per tick), so
        //    overhead is modest. Gated internally by LogUtil level checks.
        setupCommandSchedulerHooks();
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
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        //
        // No manual timing: AdvantageKit auto-logs LoggedRobot/UserCodeMS, LoggedRobot/FullCycleMS,
        // and LoggedRobot/LogPeriodicMS, and WPILib Tracer prints per-subsystem + per-command
        // breakdowns on overruns (captured in the WPILog `messages` stream). That's enough for
        // post-match diagnosis.
        CommandScheduler.getInstance().run();
        container.getTuningPeriodic().run();
        container.logSwerveCurrent();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Clear latched motor controller control requests (e.g. a stale
        // MOTION_MAGIC target on the intake arm) so they don't resume
        // driving the moment the robot is re-enabled. See
        // RobotContainer.onDisabledInit() for details.
        if (container != null) {
            container.onDisabledInit();
        }
    }

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
        // container.onTeleopInit(autonomousCommand == null);
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
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    /**
     * Sets up CommandScheduler hooks for command lifecycle logging.
     * <ul>
     *   <li>{@code Commands/Running/{name}} (ESSENTIAL) — boolean, active/inactive.
     *       Visible in AdvantageScope as a timeline so we can correlate match events
     *       with which commands were running.</li>
     *   <li>{@code Commands/Duration/{name}} (DIAGNOSTIC) — how long each command ran
     *       on finish. Useful for catching commands that overshot their expected time.</li>
     * </ul>
     * Callbacks fire on command state transitions only (not per tick).
     */
    private void setupCommandSchedulerHooks() {
        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.onCommandInitialize(command -> {
            commandStartTimes.put(command, Timer.getFPGATimestamp());
            log(ESSENTIAL, "Commands/Running/" + command.getName(), true);
        });

        scheduler.onCommandFinish(command -> {
            Double startTime = commandStartTimes.remove(command);
            if (startTime != null) {
                double durationMs = (Timer.getFPGATimestamp() - startTime) * 1000.0;
                log(DIAGNOSTIC, "Commands/Duration/" + command.getName(), durationMs);
            }
            log(ESSENTIAL, "Commands/Running/" + command.getName(), false);
        });

        scheduler.onCommandInterrupt(command -> {
            commandStartTimes.remove(command);
            log(ESSENTIAL, "Commands/Running/" + command.getName(), false);
        });
    }

    /**
     * Detect whether a FAT32 USB stick is mounted at the roboRIO's conventional path.
     * Guards {@code SignalLogger.start()} so it doesn't fill RIO onboard flash when no
     * stick is present.
     */
    private static boolean hasUsbStorage() {
        return new File("/U").exists() || new File("/media/sda1").exists();
    }
}
