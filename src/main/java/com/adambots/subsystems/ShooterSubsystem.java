package com.adambots.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.adambots.Constants.ShooterConstants;
import com.adambots.RobotMap;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    private double lobShotRPSCached = ShooterConstants.kLobShotRPS;

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

        rightFlywheel.setStrictFollower(RobotMap.kShooterMotor2Port, true);
        rightFlywheel.setBrakeMode(false);
    }

    private void initializeInterpolationTable() {
        for (double[] entry : ShooterConstants.kDefaultInterpolationTable) {
            interpolationTable.put(entry[0], entry[1]);
        }
    }

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setFlywheelPID(double p, double i, double d, double ff) {
        leftFlywheel.setPID(0, p, i, d, ff);
    }

    public void setFlywheelTolerance(double rps) {
        flywheelToleranceCached = rps;
    }

    public void setLobShotRPS(double rps) {
        lobShotRPSCached = rps;
    }

    public void rebuildInterpolationTable(double[][] entries) {
        interpolationTable.clear();
        for (double[] entry : entries) {
            interpolationTable.put(entry[0], entry[1]);
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

    /**
     * Map throttle (-1.0→1.0) to the tested RPS range.
     * Forward (away from driver, -1.0) = max RPS, back (toward driver, 1.0) = min RPS.
     */
    public double throttleToRPS(double throttle) {
        double normalized = (-throttle + 1.0) / 2.0; // 0.0→1.0 where forward=1.0
        return ShooterConstants.kMinRPS + normalized * (ShooterConstants.kMaxRPS - ShooterConstants.kMinRPS);
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

    public Command spinUpCommand(DoubleSupplier rpsSupplier) {
        return runEnd(() -> setFlywheelRPS(rpsSupplier.getAsDouble()), this::stopFlywheel)
            .withName("Spin Up (dynamic)");
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
