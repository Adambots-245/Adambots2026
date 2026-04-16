package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import static edu.wpi.first.units.Units.Degrees;

import com.adambots.Constants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.sensors.BaseAbsoluteEncoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem with Motion Magic position control via onboard motor controller.
 * Uses a 10-turn potentiometer for absolute position sensing at boot.
 * Tracking uses pose-based bearing to hardcoded hub center (gyro + odometry at 50 Hz).
 */
public class TurretSubsystem extends SubsystemBase {

    /** Tracking state for telemetry. CAMERA = actively tracking hub (pose-based). */
    enum TrackingMode { HOLD, CAMERA, SWEEP, JOG }

    private final BaseMotor turretMotor;
    private final BaseAbsoluteEncoder turretPot;

    private double trackingToleranceDeg = TurretTrackingConstants.kTrackingToleranceDeg;

    // Track last setpoint for isAtTarget()
    private double lastSetpointDegrees = TurretConstants.kTurretForwardDegrees;

    // Potentiometer calibration: pot reading at turret 0° and 180° (tunable via dashboard)
    private double potAtZeroDeg = TurretConstants.kTurretPotAtZeroDeg;
    private double potAtMaxDeg = TurretConstants.kTurretPotAtMaxDeg;

    // Auto-track toggle (driver opts in via Button 5)
    private boolean autoTrackEnabled = false;
    private boolean wasAutoTracking = false;
    private double holdAngleDegrees = TurretConstants.kTurretForwardDegrees;

    // Current tracking mode for telemetry
    private TrackingMode trackingMode = TrackingMode.HOLD;

    // Scan direction for SWEEP mode: +1 = toward max, -1 = toward zero
    private int scanDirection = 1;


    // Throttled tracking diagnostics (1 Hz)
    private double lastTrackLogTime = 0;
    private String lastTrackAction = "INIT";

    public TurretSubsystem(BaseMotor turretMotor, BaseAbsoluteEncoder turretPot) {
        this.turretMotor = turretMotor;
        this.turretPot = turretPot;
        configureMotors();

        // Seed motor encoder from potentiometer absolute position
        double absoluteDeg = getPotAngleDegrees();
        double rotations = (absoluteDeg / 360.0) * TurretConstants.kTurretMotorGearRatio;
        turretMotor.setPosition(rotations);
    }

    private void configureMotors() {
        turretMotor.configure()
            .brakeMode(true)
            .currentLimits(TurretConstants.kTurretStallCurrentLimit,
                           TurretConstants.kTurretFreeCurrentLimit, 3000)
            .motionMagic(
                RotationsPerSecond.of(TurretConstants.kTurretCruiseVelocity),
                RotationsPerSecondPerSecond.of(TurretConstants.kTurretAcceleration),
                TurretConstants.kTurretJerk)
            .apply();

        // Extended PID with feedforward gains (kV, kS, kA, kG).
        // kS is critical for turret: it adds a constant voltage in the direction
        // of error to overcome static friction from the 3D-printed gear mesh and
        // cable tray. Without it, the PID hovers at the friction breakaway
        // boundary and buzzes.
        turretMotor.setPID(0,
                TurretConstants.kTurretP, TurretConstants.kTurretI,
                TurretConstants.kTurretD,
                TurretConstants.kTurretKV, TurretConstants.kTurretKS,
                TurretConstants.kTurretKA, TurretConstants.kTurretKG);

        // Soft limits: firmware-level safety rail. The rotor encoder is
        // seeded at boot from the pot (getPotAngleDegrees() → motor rotations),
        // and the calibrated range [0°, kTurretMaxDegrees] maps to
        // [0, kTurretMaxDegrees/360 × kTurretMotorGearRatio] motor rotations.
        // Jog commands use percent output which bypasses the software clamp
        // in setTurretAngle(), so we need this firmware-level rail as a
        // backup. A small margin (kTurretSoftLimitMarginDeg) prevents the
        // limit from triggering during normal motion while still catching
        // runaway scenarios.
        double marginMotorRot =
            (TurretConstants.kTurretSoftLimitMarginDeg / 360.0)
            * TurretConstants.kTurretMotorGearRatio;
        double forwardRot =
            (TurretConstants.kTurretMaxDegrees / 360.0)
            * TurretConstants.kTurretMotorGearRatio
            + marginMotorRot;
        double reverseRot = -marginMotorRot;
        turretMotor.configureSoftLimits(forwardRot, reverseRot, true);
    }

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setTurretPID(double p, double i, double d,
                             double kV, double kS, double kA, double kG) {
        turretMotor.setPID(0, p, i, d, kV, kS, kA, kG);
    }

    public void setTrackingTolerance(double deg) {
        trackingToleranceDeg = deg;
    }

    public void setPotAtZeroDeg(double deg) {
        potAtZeroDeg = deg;
    }

    public void setPotAtMaxDeg(double deg) {
        potAtMaxDeg = deg;
    }


    public void setMotionMagicProfile(double cruiseRPS, double accelRPSPerSec) {
        turretMotor.configure()
            .motionMagic(
                RotationsPerSecond.of(cruiseRPS),
                RotationsPerSecondPerSecond.of(accelRPSPerSec),
                0)
            .apply();
    }

    // ==================== Manual Jog Helper ====================

    /**
     * Handles manual joystick jog override. Call at the top of any tracking
     * command's execute loop. Returns true if jog is active (caller should
     * skip tracking logic), false if jog is inactive (proceed with tracking).
     */
    boolean handleManualJog(DoubleSupplier manualJogInput) {
        double jogRaw = manualJogInput.getAsDouble();
        double jogDeadbanded = MathUtil.applyDeadband(jogRaw, TurretConstants.kTurretJogDeadband);
        if (jogDeadbanded != 0.0) {
            double jogOutput = Math.copySign(jogDeadbanded * jogDeadbanded, jogDeadbanded)
                * TurretConstants.kTurretJogMaxPercent;
            turretMotor.set(jogOutput);
            trackingMode = TrackingMode.JOG;
            lastTrackAction = String.format("JOG %.0f%%", jogOutput * 100);
            holdAngleDegrees = getTurretAngleDegrees();
            return true;
        }
        if (trackingMode == TrackingMode.JOG) {
            holdAngleDegrees = getTurretAngleDegrees();
            lastSetpointDegrees = holdAngleDegrees;
            trackingMode = TrackingMode.HOLD;
        }
        return false;
    }

    // ==================== Pose-to-Turret Conversion ====================

    /**
     * Converts a robot-relative bearing to the hub into a turret angle.
     * The shooter points backward at kTurretForwardDegrees (99°), so
     * hub at ±180° robot-relative maps to turret 99°.
     *
     * <p>The result is NOT clamped — callers should clamp to [0, kTurretMaxDegrees].
     * Out-of-range values indicate the hub is unreachable by the turret.
     *
     * @param poseAngleDeg robot-relative bearing to hub (0°=front, ±180°=back)
     * @return turret angle in degrees (may be out of range)
     */
    static double poseAngleToTurretAngle(double poseAngleDeg) {
        // kTurretForwardDegrees (99°) = shooter points straight back (toward hub).
        // Turret rotation is OPPOSITE to WPILib bearing convention:
        //   WPILib: +angle = CCW (left)
        //   Turret: +angle = toward left (viewed from back), which is CCW from back
        //
        // Formula: 99 + (180 - robotRelative) = 279 - robotRelative
        //   Hub behind (180°) → 279 - 180 = 99° → straight back ✓
        //   Hub left   (90°)  → 279 - 90  = 189° → turret left ✓
        //   Hub right  (-90°) → 279 + 90  = 369° → normalize → 9° → turret right ✓
        //   Hub front  (0°)   → 279 - 0   = 279° → normalize → -81° → unreachable ✓
        double raw = TurretConstants.kTurretForwardDegrees + 180.0 - poseAngleDeg;
        // Normalize result to be near the turret's valid range.
        while (raw > TurretConstants.kTurretForwardDegrees + 180) raw -= 360;
        while (raw < TurretConstants.kTurretForwardDegrees - 180) raw += 360;
        return raw;
    }

    // ==================== Turret Control ====================

    public void setTurretAngle(double degrees) {
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double rotations = (degrees / 360.0) * TurretConstants.kTurretMotorGearRatio;
        turretMotor.set(ControlMode.MOTION_MAGIC, rotations);
        // turretMotor.set(ControlMode.POSITION, rotations);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public double getTurretAngleDegrees() {
        return (turretMotor.getPosition() / TurretConstants.kTurretMotorGearRatio) * 360.0;
    }

    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(getTurretAngleDegrees() - lastSetpointDegrees) < toleranceDeg;
    }

    // ==================== Potentiometer ====================

    /** Returns the raw pot reading in degrees (0-3600 for a 10-turn pot). */
    public double getRawPotDegrees() {
        return turretPot.getPosition().in(Degrees);
    }

    /** Converts raw pot reading to turret angle in degrees [0, kTurretMaxDegrees]. */
    public double getPotAngleDegrees() {
        double range = potAtMaxDeg - potAtZeroDeg;
        if (range == 0) return 0.0;
        double rawDeg = getRawPotDegrees();
        return (rawDeg - potAtZeroDeg) / range * TurretConstants.kTurretMaxDegrees;
    }

    // ==================== Triggers ====================

    public Trigger isAtTargetTrigger() {
        return new Trigger(() -> isAtTarget(trackingToleranceDeg));
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        // Pot seeds motor encoder on construction — no continuous re-sync needed.
        // Continuous re-sync fights the PID controller and causes oscillation.

        if (Constants.CURRENT_LOGGING) {
            Logger.recordOutput("Turret/Current", turretMotor.getCurrentDraw().in(Amps));
        }

        // Dashboard telemetry — only when tuning to reduce bandwidth
        if (Constants.TUNING_ENABLED) {
            double currentAngle = getTurretAngleDegrees();
            SmartDashboard.putNumber("Turret/Angle (deg)", currentAngle);
            SmartDashboard.putNumber("Turret/Pot Raw (deg)", getRawPotDegrees());
            SmartDashboard.putNumber("Turret/Pot Turret (deg)", getPotAngleDegrees());
            SmartDashboard.putBoolean("Turret/AutoTrack", autoTrackEnabled);
            SmartDashboard.putNumber("Turret/Setpoint (deg)", lastSetpointDegrees);
            SmartDashboard.putNumber("Turret/Error (deg)", lastSetpointDegrees - currentAngle);
            SmartDashboard.putString("Turret/TrackingMode", trackingMode.name());
            SmartDashboard.putNumber("Turret/ScanDirection", scanDirection);
        }
    }

    // ==================== Command Factories ====================

    /** Continuously aims turret at angle from supplier (for vision tracking). */
    public Command aimTurretCommand(DoubleSupplier angleSupplier) {
        return run(() -> setTurretAngle(angleSupplier.getAsDouble()))
            .withName("Aim Turret Dynamic");
    }

    /**
     * Manual jog — applies a constant percent-output voltage while held, and
     * zero when released. The {@code speed} parameter is a direction multiplier
     * (±1 typically); the actual magnitude is {@code kTurretJogPercent}.
     *
     * <p>Deliberately uses percent output instead of Motion Magic. For a
     * "hold-to-move" interaction, Motion Magic is the wrong tool — setting a
     * new target every scheduler tick causes continuous trajectory resets
     * and audible buzz. A constant voltage command is smooth and natural.
     * Go-to-angle commands (aim, forward, hold, pose-track, manual align)
     * correctly use Motion Magic via setTurretAngle() and are unchanged.
     *
     * <p>Position bounds during jog are enforced at the firmware level by
     * the soft limits configured in configureMotors().
     */
    public Command scanCommand(double speed) {
        return runEnd(
            () -> turretMotor.set(speed * TurretConstants.kTurretJogPercent),
            () -> {
                turretMotor.set(0);
                holdAngleDegrees = getTurretAngleDegrees();
            }
        ).withName("Scan Turret");
    }

    /** Holds current turret angle. */
    public Command holdPositionCommand() {
        return runOnce(() -> setTurretAngle(getTurretAngleDegrees()))
            .withName("Hold Position");
    }

    public Command stopTurretCommand() {
        return Commands.runOnce(this::stopTurret, this)
            .withName("Stop Turret");
    }

    public Command toggleAutoTrackCommand() {
        return Commands.runOnce(() -> autoTrackEnabled = !autoTrackEnabled)
            .withName("Toggle Auto-Track");
    }

    public void setAutoTrack(boolean enabled) {
        autoTrackEnabled = enabled;
    }

    public boolean isAutoTrackEnabled() {
        return autoTrackEnabled;
    }

    // ==================== Pose-Lock Tracker Command ====================

    /**
     * Pose-lock tracker — uses odometry + gyro for smooth continuous tracking.
     * Computes turret angle from the hardcoded hub center and robot pose at
     * 50 Hz (gyro rate). No camera dependency for tracking — the hub is a
     * known field coordinate.
     *
     * <p>Includes angular velocity lead compensation to anticipate robot
     * rotation, and a low-pass filter to smooth Motion Magic commands.
     * Falls back to slow sweep if pose is invalid (at field origin).
     *
     * @param cameraYaw      unused (kept for interface compatibility)
     * @param hubVisible     unused (kept for interface compatibility)
     * @param hubFresh       unused (kept for interface compatibility)
     * @param poseSupplier   robot pose (for bearing computation)
     * @param hubCenter      hub center position on the field (hardcoded)
     * @param robotAngularVelDegPerSec robot yaw rate for rotation lead compensation
     * @param manualJogInput raw joystick axis [-1, 1] for manual override
     */
    public Command poseTrackCommand(
            DoubleSupplier cameraYaw,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Translation2d> hubCenter,
            DoubleSupplier robotAngularVelDegPerSec,
            DoubleSupplier manualJogInput) {

        // Mutable state for the lock phase
        boolean[] locked = { false };
        double[] smoothedAngle = { TurretConstants.kTurretForwardDegrees };

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            scanDirection = 1;
            locked[0] = false;
            smoothedAngle[0] = getTurretAngleDegrees();
        }, this)
        .andThen(run(() -> {
            if (handleManualJog(manualJogInput)) return;

            // Auto-track toggle (Button 5) — same as autoTrackCommand
            if (!autoTrackEnabled) {
                if (wasAutoTracking) {
                    holdAngleDegrees = getTurretAngleDegrees();
                    wasAutoTracking = false;
                }
                trackingMode = TrackingMode.HOLD;
                setTurretAngle(holdAngleDegrees);
                return;
            }
            wasAutoTracking = true;

            double currentAngle = getTurretAngleDegrees();
            var pose = poseSupplier.get();
            var hub = hubCenter.get();

            // Compute turret pivot position in field frame (not robot center).
            // The turret pivot is offset from the robot center — using robot
            // center introduces aim error at close range (~7° at 1.5m).
            double headingRad = pose.getRotation().getRadians();
            double pivotX = pose.getX()
                + TurretConstants.kTurretPivotX * Math.cos(headingRad)
                - TurretConstants.kTurretPivotY * Math.sin(headingRad);
            double pivotY = pose.getY()
                + TurretConstants.kTurretPivotX * Math.sin(headingRad)
                + TurretConstants.kTurretPivotY * Math.cos(headingRad);

            // Compute bearing from turret pivot to hub center
            double dx = hub.getX() - pivotX;
            double dy = hub.getY() - pivotY;
            double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
            double robotHeading = pose.getRotation().getDegrees();
            double robotRelative = worldBearing - robotHeading;
            double poseTurretAngle = poseAngleToTurretAngle(robotRelative);

            // Pure pose-based tracking: always point at the hardcoded hub center.
            // Bearing is computed from gyro + odometry at 50 Hz — no camera needed.
            if (pose.getTranslation().getNorm() > 1.0) {
                // Valid pose — point at hub center
                double clampedAngle = MathUtil.clamp(poseTurretAngle, 0, TurretConstants.kTurretMaxDegrees);

                if (poseTurretAngle < -10 || poseTurretAngle > TurretConstants.kTurretMaxDegrees + 10) {
                    // Hub is unreachable (in front of robot) — face forward
                    trackingMode = TrackingMode.HOLD;
                    setTurretAngle(TurretConstants.kTurretForwardDegrees);
                    lastTrackAction = "OUT OF RANGE";
                } else {
                    trackingMode = TrackingMode.CAMERA;
                    locked[0] = true;
                    // Angular velocity lead: anticipate where the hub will be
                    // by the time Motion Magic finishes its profile. The turret
                    // convention is inverted (higher angle = left), so negate omega.
                    double angVelLead = -robotAngularVelDegPerSec.getAsDouble()
                        * TurretTrackingConstants.kAngularVelLeadTime;
                    // Low-pass filter: smooth the command to reduce MM trajectory
                    // restarts. 0.5 = 50% new target per cycle → settles in ~3 cycles (60ms).
                    smoothedAngle[0] += (clampedAngle + angVelLead - smoothedAngle[0]) * 0.5;
                    setTurretAngle(smoothedAngle[0]);
                    lastTrackAction = String.format("TRACK %.1f° lead=%.1f", smoothedAngle[0], angVelLead);
                }
            } else {
                // No valid pose (at origin) — slow sweep until odom cameras initialize
                locked[0] = false;
                trackingMode = TrackingMode.SWEEP;
                if (currentAngle >= TurretConstants.kTurretMaxDegrees
                        - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                turretMotor.set(scanDirection * TurretConstants.kTurretJogPercent * 0.4);
                lastTrackAction = "SWEEP dir=" + scanDirection;
            }

            // Logging — gated behind LOGGING_ENABLED for competition performance
            if (Constants.LOGGING_ENABLED) {
                Logger.recordOutput("Turret/TrackMode", trackingMode.name());
                Logger.recordOutput("Turret/TrackAction", lastTrackAction);
                Logger.recordOutput("Turret/CurrentAngle", currentAngle);
                Logger.recordOutput("Turret/PoseTurretAngle", poseTurretAngle);
                Logger.recordOutput("Turret/WorldBearing", worldBearing);
                Logger.recordOutput("Turret/RobotRelative", robotRelative);
                Logger.recordOutput("Turret/RobotHeading", robotHeading);
                Logger.recordOutput("Turret/Locked", locked[0]);
                Logger.recordOutput("Turret/HubVisible", hubVisible.getAsBoolean());
                Logger.recordOutput("Turret/HubFresh", hubFresh.getAsBoolean());
            }
        }))
        .finallyDo(interrupted -> {
            turretMotor.set(0);
            holdAngleDegrees = getTurretAngleDegrees();
        })
        .withName("Pose Track");
    }

    // ==================== Manual Align Command ====================

    /**
     * One-shot manual align using pose-based tracking. Same logic as
     * poseTrackCommand but without the autoTrackEnabled gate — runs
     * immediately when the button is pressed. Holds position on release.
     *
     * @param poseSupplier   robot pose (for bearing computation)
     * @param hubCenter      hub center position on the field
     * @param robotAngularVelDegPerSec robot yaw rate for rotation lead
     */
    public Command manualAlignCommand(
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Translation2d> hubCenter,
            DoubleSupplier robotAngularVelDegPerSec) {

        double[] smoothedAngle = { TurretConstants.kTurretForwardDegrees };

        return Commands.runOnce(() -> {
            smoothedAngle[0] = getTurretAngleDegrees();
        }, this)
        .andThen(run(() -> {
            double currentAngle = getTurretAngleDegrees();
            var pose = poseSupplier.get();
            var hub = hubCenter.get();

            if (pose.getTranslation().getNorm() > 1.0) {
                double headingRad = pose.getRotation().getRadians();
                double pivotX = pose.getX()
                    + TurretConstants.kTurretPivotX * Math.cos(headingRad)
                    - TurretConstants.kTurretPivotY * Math.sin(headingRad);
                double pivotY = pose.getY()
                    + TurretConstants.kTurretPivotX * Math.sin(headingRad)
                    + TurretConstants.kTurretPivotY * Math.cos(headingRad);

                double dx = hub.getX() - pivotX;
                double dy = hub.getY() - pivotY;
                double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
                double robotHeading = pose.getRotation().getDegrees();
                double robotRelative = worldBearing - robotHeading;
                double poseTurretAngle = poseAngleToTurretAngle(robotRelative);
                double clampedAngle = MathUtil.clamp(poseTurretAngle, 0, TurretConstants.kTurretMaxDegrees);

                double angVelLead = -robotAngularVelDegPerSec.getAsDouble()
                    * TurretTrackingConstants.kAngularVelLeadTime;
                smoothedAngle[0] += (clampedAngle + angVelLead - smoothedAngle[0]) * 0.5;
                trackingMode = TrackingMode.CAMERA;
                setTurretAngle(smoothedAngle[0]);
            } else {
                // No valid pose — slow sweep
                trackingMode = TrackingMode.SWEEP;
                if (currentAngle >= TurretConstants.kTurretMaxDegrees
                        - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                turretMotor.set(scanDirection * TurretConstants.kTurretJogPercent * 0.4);
            }
        }))
        .finallyDo(interrupted -> {
            holdAngleDegrees = getTurretAngleDegrees();
            setTurretAngle(holdAngleDegrees);
        })
        .withName("Manual Align");
    }

}
