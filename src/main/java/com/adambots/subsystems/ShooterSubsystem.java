package com.adambots.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.adambots.logging.LogUtil.ESSENTIAL;
import static com.adambots.logging.LogUtil.log;

import com.adambots.Constants.ShooterConstants;
import com.adambots.RobotMap;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Flywheel-only shooter subsystem with PID velocity control.
 * Uses an interpolation table to map distance (meters) to flywheel RPS.
 * All closed-loop control runs on the motor controller at 1kHz.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final BaseMotor leftFlywheel;
    private final BaseMotor rightFlywheel;
    private final Supplier<Pose2d> robotPose;

    private boolean idleEnabled = false;
    private boolean shotBoostActive = false;

    private double targetRPS = 0;
    private double flywheelToleranceCached = ShooterConstants.kFlywheelToleranceRPS;

    // Interpolation table for distance -> RPS lookup
    private final InterpolatingDoubleTreeMap interpolationTable = new InterpolatingDoubleTreeMap();

    private double lobShotRPSCached = ShooterConstants.kLobShotRPS;

    public ShooterSubsystem(BaseMotor leftFlywheel, BaseMotor rightFlywheel, Supplier<Pose2d> robotPose) {
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.robotPose = robotPose;
        configureMotors();
        initializeInterpolationTable();
    }

    private void configureMotors() {
        leftFlywheel.configure()
            .pid(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelI,
                 ShooterConstants.kFlywheelD, ShooterConstants.kFlywheelFF)
            .brakeMode(false)  // coast for flywheels
            .inverted(true)
            .currentLimits(ShooterConstants.kFlywheelStatorCurrentLimit,
                           ShooterConstants.kFlywheelSupplyCurrentLimit, 3000)
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
        double effectiveRPS = rps + (shotBoostActive ? ShooterConstants.kShotBoostRPS : 0);
        leftFlywheel.set(ControlMode.VELOCITY_FOC_TORQUE, effectiveRPS * ShooterConstants.kFlywheelDirection);
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
        return MathUtil.clamp(interpolationTable.get(distanceMeters),
            ShooterConstants.kMinRPS, ShooterConstants.kMaxRPS);
    }

    // ==================== Time of Flight ====================

    /** Flywheel radius in meters (2 inches). */
    private static final double FLYWHEEL_RADIUS_M = 0.0508;
    /** Hood angle in radians (60°). */
    private static final double HOOD_ANGLE_RAD = Math.toRadians(60.0);
    /** Ball exit velocity as fraction of flywheel surface speed. */
    private static final double EXIT_VELOCITY_MULTIPLIER = 0.85;

    /**
     * Estimates ball time-of-flight to the hub at a given distance.
     * Computed from: distance / (exitVelocity × cos(hoodAngle)).
     *
     * @param distanceMeters distance to hub
     * @return estimated TOF in seconds (clamped to 0.1-1.0s)
     */
    public double getEstimatedTOF(double distanceMeters) {
        double rps = getRPSFromTable(distanceMeters);
        double surfaceSpeed = rps * 2 * Math.PI * FLYWHEEL_RADIUS_M; // m/s
        double exitSpeed = surfaceSpeed * EXIT_VELOCITY_MULTIPLIER;
        double horizontalSpeed = exitSpeed * Math.cos(HOOD_ANGLE_RAD);
        if (horizontalSpeed < 0.1) return 1.0; // safety fallback
        return MathUtil.clamp(distanceMeters / horizontalSpeed, 0.1, 1.0);
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

    // ==================== Field Zone ====================

    /**
     * Returns true when the robot is between its alliance wall and the hub.
     * Once the robot passes the hub (toward mid-field), tracking stops.
     * Hub X positions from 2026 AprilTag field layout.
     */
    public boolean isInShootingZone() {
        double robotX = robotPose.get().getX();
        boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        return isRed ? robotX > ShooterConstants.kRedShootingZoneMinX
                     : robotX < ShooterConstants.kBlueShootingZoneMaxX;
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

    // ==================== Idle Pre-Spin ====================

    public void setIdleEnabled(boolean enabled) { idleEnabled = enabled; }
    public boolean isIdleEnabled() { return idleEnabled; }

    public void setShotBoost(boolean active) { shotBoostActive = active; }
    public boolean isShotBoostActive() { return shotBoostActive; }

    /**
     * Default command: maintains low idle RPM when enabled, stops when disabled.
     * Automatically interrupted by any shoot command (which requires this subsystem),
     * and resumes when shooting ends.
     */
    @Override
    public void periodic() {
        // ESSENTIAL: match-critical shooter state. Post-match we need to answer
        // "did we spin up to the right target?", "was it at speed when we shot?",
        // and "how much current did each flywheel draw?".
        log(ESSENTIAL, "Shooter/TargetRPS", targetRPS);
        log(ESSENTIAL, "Shooter/LeftActualRPS", getLeftRPS());
        log(ESSENTIAL, "Shooter/RightActualRPS", getRightRPS());
        log(ESSENTIAL, "Shooter/AtSpeed", isAtSpeed());
        log(ESSENTIAL, "Shooter/ShotBoost", shotBoostActive);
        log(ESSENTIAL, "Shooter/LeaderCurrent", leftFlywheel.getCurrentDraw().in(Amps));
        log(ESSENTIAL, "Shooter/FollowerCurrent", rightFlywheel.getCurrentDraw().in(Amps));
    }

    public Command idleCommand() {
        return run(() -> {
            if (idleEnabled) {
                setFlywheelRPS(ShooterConstants.kIdleRPS);
            } else {
                stopFlywheel();
            }
        }).finallyDo(() -> stopFlywheel())
          .withName("Flywheel Idle");
    }
}
