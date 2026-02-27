package com.adambots.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.adambots.Constants;
import com.adambots.Constants.ShooterConstants;
import com.adambots.RobotMap;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Flywheel-only shooter subsystem with PID velocity control.
 * Uses an interpolation table to map distance (meters) to flywheel RPS.
 * All closed-loop control runs on the motor controller at 1kHz.
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {

    private final BaseMotor leftFlywheel;
    private final BaseMotor rightFlywheel;

    private double targetRPS = 0;
    private double flywheelToleranceCached = ShooterConstants.kFlywheelToleranceRPS;

    // Interpolation table for distance -> RPS lookup
    private final InterpolatingDoubleTreeMap interpolationTable = new InterpolatingDoubleTreeMap();

    // ==================== Tunable GenericEntry Fields ====================
    private GenericEntry flywheelPEntry;
    private GenericEntry flywheelIEntry;
    private GenericEntry flywheelDEntry;
    private GenericEntry flywheelFFEntry;
    private GenericEntry flywheelToleranceEntry;

    // Interpolation table tunable entries (5 rows of distance + RPS)
    private final GenericEntry[] tableDistanceEntries = new GenericEntry[5];
    private final GenericEntry[] tableRPSEntries = new GenericEntry[5];

    // Lob shot tunable
    private GenericEntry lobShotRPSEntry;
    private double lobShotRPSCached = ShooterConstants.kLobShotRPS;

    // Last-applied PID values to avoid flooding CAN bus
    private double lastFlywheelP, lastFlywheelI, lastFlywheelD, lastFlywheelFF;

    public ShooterSubsystem(BaseMotor leftFlywheel, BaseMotor rightFlywheel) {
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        configureMotors();
        initializeInterpolationTable();
    }

    private void configureMotors() {
        leftFlywheel.configure()
            .pid(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelI,
                 ShooterConstants.kFlywheelD, ShooterConstants.kFlywheelFF)
            .brakeMode(false)  // coast for flywheels
            .inverted(true)
            .currentLimits(ShooterConstants.kFlywheelStallCurrentLimit,
                           ShooterConstants.kFlywheelFreeCurrentLimit, 3000)
            .apply();

        rightFlywheel.setStrictFollower(RobotMap.kShooterLeftPort, true);
        rightFlywheel.setBrakeMode(false);

        lastFlywheelP = ShooterConstants.kFlywheelP;
        lastFlywheelI = ShooterConstants.kFlywheelI;
        lastFlywheelD = ShooterConstants.kFlywheelD;
        lastFlywheelFF = ShooterConstants.kFlywheelFF;
    }

    private void initializeInterpolationTable() {
        for (double[] entry : ShooterConstants.kDefaultInterpolationTable) {
            interpolationTable.put(entry[0], entry[1]);
        }
    }

    // ==================== Tunable Setup ====================

    /**
     * Registers flywheel tunable GenericEntry fields on the current Dash tab.
     * Call after Dash.useTab() in RobotContainer.
     * @param pos position tracker {col, row}, updated in place
     * @param cols max columns per row before wrapping
     */
    public void setupFlywheelTunables(int[] pos, int cols) {
        if (!Constants.TUNING_ENABLED) return;
        flywheelPEntry = Dash.addTunable("Flywheel kP", ShooterConstants.kFlywheelP, pos[0], pos[1]);
        advance(pos, cols);
        flywheelIEntry = Dash.addTunable("Flywheel kI", ShooterConstants.kFlywheelI, pos[0], pos[1]);
        advance(pos, cols);
        flywheelDEntry = Dash.addTunable("Flywheel kD", ShooterConstants.kFlywheelD, pos[0], pos[1]);
        advance(pos, cols);
        flywheelFFEntry = Dash.addTunable("Flywheel kF", ShooterConstants.kFlywheelFF, pos[0], pos[1]);
        advance(pos, cols);
        flywheelToleranceEntry = Dash.addTunable("Flywheel Tolerance (RPS)", ShooterConstants.kFlywheelToleranceRPS, pos[0], pos[1]);
        advance(pos, cols);

        // Table distances in one row, RPS values directly below
        newRow(pos);
        for (int i = 0; i < 5; i++) {
            tableDistanceEntries[i] = Dash.addTunable(
                "Table Dist " + (i + 1), ShooterConstants.kDefaultInterpolationTable[i][0], pos[0], pos[1]);
            advance(pos, cols);
        }
        newRow(pos);
        for (int i = 0; i < 5; i++) {
            tableRPSEntries[i] = Dash.addTunable(
                "Table RPS " + (i + 1), ShooterConstants.kDefaultInterpolationTable[i][1], pos[0], pos[1]);
            advance(pos, cols);
        }

        // Lob shot RPS
        newRow(pos);
        lobShotRPSEntry = Dash.addTunable("Lob Shot RPS", ShooterConstants.kLobShotRPS, pos[0], pos[1]);
        advance(pos, cols);
    }

    private static void advance(int[] pos, int cols) {
        pos[0]++;
        if (pos[0] >= cols) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    private static void newRow(int[] pos) {
        if (pos[0] != 0) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    // ==================== Flywheel Control ====================

    public void setFlywheelRPS(double rps) {
        targetRPS = rps;
        leftFlywheel.set(ControlMode.VELOCITY, rps * ShooterConstants.kFlywheelDirection);
    }

    public void stopFlywheel() {
        targetRPS = 0;
        leftFlywheel.set(0);
    }

    /**
     * Spin the flywheel for a given distance using interpolation table lookup.
     */
    public void spinForDistance(double meters) {
        setFlywheelRPS(getRPSFromTable(meters));
    }

    public double getRPSFromTable(double distanceMeters) {
        return interpolationTable.get(distanceMeters);
    }

    // ==================== Telemetry Getters ====================

    public double getLeftRPS() {
        return leftFlywheel.getVelocity().in(RotationsPerSecond);
    }

    public double getRightRPS() {
        return rightFlywheel.getVelocity().in(RotationsPerSecond);
    }

    public double getTargetRPS() {
        return targetRPS;
    }

    public double lobShotRPS() {
        return lobShotRPSCached;
    }

    public boolean isAtSpeed() {
        return targetRPS > 0 && Math.abs(Math.abs(getLeftRPS()) - targetRPS) < flywheelToleranceCached;
    }

    // ==================== Triggers ====================

    public Trigger isAtSpeedTrigger() {
        return new Trigger(this::isAtSpeed);
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        // Apply tunable PID values only when changed
        if (flywheelPEntry != null) {
            double p = flywheelPEntry.getDouble(ShooterConstants.kFlywheelP);
            double i = flywheelIEntry.getDouble(ShooterConstants.kFlywheelI);
            double d = flywheelDEntry.getDouble(ShooterConstants.kFlywheelD);
            double f = flywheelFFEntry.getDouble(ShooterConstants.kFlywheelFF);

            if (p != lastFlywheelP || i != lastFlywheelI || d != lastFlywheelD || f != lastFlywheelFF) {
                leftFlywheel.setPID(0, p, i, d, f);
                lastFlywheelP = p;
                lastFlywheelI = i;
                lastFlywheelD = d;
                lastFlywheelFF = f;
            }

            flywheelToleranceCached = flywheelToleranceEntry.getDouble(ShooterConstants.kFlywheelToleranceRPS);

            if (lobShotRPSEntry != null) {
                lobShotRPSCached = lobShotRPSEntry.getDouble(ShooterConstants.kLobShotRPS);
            }
        }

        // Rebuild interpolation table from tunable entries
        if (tableDistanceEntries[0] != null) {
            interpolationTable.clear();
            for (int idx = 0; idx < 5; idx++) {
                double dist = tableDistanceEntries[idx].getDouble(
                    ShooterConstants.kDefaultInterpolationTable[idx][0]);
                double rps = tableRPSEntries[idx].getDouble(
                    ShooterConstants.kDefaultInterpolationTable[idx][1]);
                interpolationTable.put(dist, rps);
            }
        }
    }

    // ==================== Command Factories ====================

    /** Spin up to the middle table entry's RPS (default). */
    public Command spinUpCommand() {
        return runEnd(
            () -> setFlywheelRPS(ShooterConstants.kDefaultInterpolationTable[2][1]),
            this::stopFlywheel
        ).withName("Spin Up");
    }

    public Command spinUpCommand(double rps) {
        return runEnd(() -> setFlywheelRPS(rps), this::stopFlywheel)
            .withName("Spin " + rps + " RPS");
    }

    /** Spin flywheel to the RPS needed for the given distance (from vision). */
    public Command spinForDistanceCommand(DoubleSupplier distanceSupplier) {
        return runEnd(
            () -> spinForDistance(distanceSupplier.getAsDouble()),
            this::stopFlywheel
        ).withName("Spin For Distance");
    }

    public Command stopFlywheelCommand() {
        return runOnce(this::stopFlywheel)
            .withName("Stop Flywheel");
    }
}
