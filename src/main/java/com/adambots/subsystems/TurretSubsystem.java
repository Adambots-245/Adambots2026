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
 * Turret subsystem with position-controlled PID via onboard motor controller.
 * Uses a 10-turn potentiometer for absolute position sensing — no calibration needed.
 * The pot seeds the motor encoder on construction and re-syncs periodically.
 */
public class TurretSubsystem extends SubsystemBase {

    /** Tracking state for telemetry. */
    enum TrackingMode { HOLD, CAMERA, SWEEP, JOG }

    private final BaseMotor turretMotor;
    private final BaseAbsoluteEncoder turretPot;

    private double trackingToleranceDeg = TurretTrackingConstants.kTrackingToleranceDeg;
    private double cameraTrackingGain = TurretTrackingConstants.kCameraTrackingGain;
    private double angularVelLeadTime = TurretTrackingConstants.kAngularVelLeadTime;

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

    // Tracking improvements: dead zone, debounce, brake, warmup
    private int trackingDebounceCount = 0;
    private int cameraBrakeFrames = 0;
    private int sweepWarmupFrames = TurretTrackingConstants.kSweepWarmupFrames;

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

    // Tracking tunable setters
    public void setCameraTrackingGain(double gain) { cameraTrackingGain = gain; }
    public void setAngularVelLeadTime(double sec) { angularVelLeadTime = sec; }

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
     * Converts a robot-relative bearing (from getYawToPoint) to a turret angle.
     * The shooter/camera faces backward, so hub at ±180° robot-relative
     * maps to turret forward (kTurretForwardDegrees).
     *
     * @param poseAngleDeg robot-relative bearing to hub (0°=front, ±180°=back)
     * @return turret angle in degrees [0, kTurretMaxDegrees]
     */
    /**
     * Converts a robot-relative bearing to the hub into a turret angle.
     * The shooter/camera faces backward, so hub at ±180° robot-relative
     * maps to turret forward (kTurretForwardDegrees = 99°).
     *
     * <p>The result is NOT clamped — callers should clamp to [0, kTurretMaxDegrees].
     * Out-of-range values indicate the hub is unreachable by the turret.
     */
    static double poseAngleToTurretAngle(double poseAngleDeg) {
        double raw = TurretConstants.kTurretForwardDegrees + poseAngleDeg - 180.0;
        // Normalize result to be near the turret's valid range.
        // Turret range is roughly [0, 256]. Center on kTurretForwardDegrees (99°).
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
     * Go-to-angle commands (aim, forward, hold, auto-track, manual align)
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

    // ==================== Vision Tracking Commands ====================

    /**
     * Auto-track command: search-then-lock system.
     * Vision layer provides sticky visibility (holdoff) so this command just needs:
     * visible → track, not visible → search.
     *
     * @param cameraAngle      turret-relative yaw offset (degrees)
     * @param hubVisible       sticky visibility — stays true during holdoff after last detection
     * @param hubFresh         raw per-frame detection — true only when PV sees hub this frame
     * @param inShootingZone   whether robot is between alliance wall and hub
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            BooleanSupplier inShootingZone) {
        return autoTrackCommand(cameraAngle, hubVisible, hubFresh, inShootingZone, () -> 0.0, () -> 0.0);
    }

    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            BooleanSupplier inShootingZone,
            DoubleSupplier robotAngularVelDegPerSec) {
        return autoTrackCommand(cameraAngle, hubVisible, hubFresh, inShootingZone, robotAngularVelDegPerSec, () -> 0.0);
    }

    /**
     * Auto-track command with integrated manual jog override.
     * When the joystick is outside the deadband, the turret switches to proportional
     * percent-output jog (squared input for fine control). When the joystick returns
     * to center, auto-track resumes seamlessly from the current position.
     *
     * @param cameraAngle              turret-relative yaw offset (degrees)
     * @param hubVisible               sticky visibility — stays true during holdoff after last detection
     * @param hubFresh                 raw per-frame detection — true only when PV sees hub this frame
     * @param inShootingZone           whether robot is between alliance wall and hub
     * @param robotAngularVelDegPerSec robot yaw rate for rotation compensation
     * @param manualJogInput           raw joystick axis [-1, 1] for proportional turret jog
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            BooleanSupplier inShootingZone,
            DoubleSupplier robotAngularVelDegPerSec,
            DoubleSupplier manualJogInput) {

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            wasAutoTracking = false;
            scanDirection = 1;
        }, this)
        .andThen(run(() -> {
            if (handleManualJog(manualJogInput)) return;

            // Auto-track toggle (Button 5)
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

            // Not in shooting zone → face forward
            if (!inShootingZone.getAsBoolean()) {
                trackingMode = TrackingMode.HOLD;
                setTurretAngle(TurretConstants.kTurretForwardDegrees);
                return;
            }

            // Angular velocity lead: compensate for robot rotation so turret holds aim.
            // WPILib convention: positive omega = CCW rotation.
            // When robot rotates CCW (+omega), the hub drifts CW in robot frame,
            // so the turret must lead in the positive-angle direction (+lead).
            // No negation needed — positive omega produces positive lead directly.
            double angVelLead = robotAngularVelDegPerSec.getAsDouble()
                * angularVelLeadTime;
            angVelLead = MathUtil.clamp(angVelLead, -8.0, 8.0);

            double currentAngle = getTurretAngleDegrees();

            if (hubVisible.getAsBoolean()) {
                // CAMERA MODE — hub visible (camera detection or pose-derived)
                if (trackingMode != TrackingMode.CAMERA) {
                    // Entering CAMERA from SWEEP/HOLD — brake first, then seed setpoint
                    lastSetpointDegrees = currentAngle;
                    cameraBrakeFrames = TurretTrackingConstants.kCameraBrakeFrames;
                    trackingDebounceCount = 0;
                    lastTrackAction = "LOCK-ON brake=" + cameraBrakeFrames;
                }
                trackingMode = TrackingMode.CAMERA;
                sweepWarmupFrames = 0;
                // Brake: stop motor to decelerate before tracking
                if (cameraBrakeFrames > 0) {
                    cameraBrakeFrames--;
                    stopTurret();
                    lastTrackAction = "BRAKING frames=" + cameraBrakeFrames;
                    return;
                }
                // Remember which side the hub is for faster reacquisition
                scanDirection = (cameraAngle.getAsDouble() >= 0) ? 1 : -1;
                // Compute target from current angle + turret-relative offset
                double camAngle = cameraAngle.getAsDouble();
                double rawTarget = currentAngle + camAngle;
                // Simple MM tracking: fresh → correct, stale → hold.
                // No rate limiter, no non-linear gain, no stale decay.
                // The gain is applied ONLY on fresh camera frames to prevent
                // stale data from being re-applied across multiple cycles.
                if (Math.abs(camAngle) < trackingToleranceDeg) {
                    trackingDebounceCount = 0;
                    setTurretAngle(lastSetpointDegrees + angVelLead);
                    lastTrackAction = "DEADZONE hold";
                } else if (rawTarget < 0 || rawTarget > TurretConstants.kTurretMaxDegrees) {
                    setTurretAngle(MathUtil.clamp(rawTarget, 0, TurretConstants.kTurretMaxDegrees));
                    lastTrackAction = "LIMIT clamp=" + String.format("%.1f", rawTarget);
                } else if (hubFresh.getAsBoolean()) {
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * cameraTrackingGain;
                    setTurretAngle(smoothed + angVelLead);
                    lastTrackAction = "TRACK fresh";
                } else {
                    // Stale: hold last setpoint, let MM finish its profile
                    setTurretAngle(lastSetpointDegrees + angVelLead);
                    lastTrackAction = "HOLD stale";
                }
            } else {
                // SWEEP: continuous smooth scan, reverse at limits
                trackingMode = TrackingMode.SWEEP;
                if (sweepWarmupFrames > 0) {
                    sweepWarmupFrames--;
                    setTurretAngle(holdAngleDegrees);
                    lastTrackAction = "WARMUP " + sweepWarmupFrames;
                    return;
                }
                if (currentAngle >= TurretConstants.kTurretMaxDegrees - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                setTurretAngle(currentAngle + scanDirection * TurretTrackingConstants.kScanStepDeg);
                lastTrackAction = "SWEEP dir=" + scanDirection;
            }

            // ==================== Structured logging (AdvantageScope) ====================
            // Always active — writes to WPILog. Lets you see exactly what the
            // auto-track loop received and what it commanded, overlaid with
            // the Vision/ signals in AdvantageScope.
            double camAngleVal = cameraAngle.getAsDouble();
            double omegaVal = robotAngularVelDegPerSec.getAsDouble();
            Logger.recordOutput("Turret/TrackMode", trackingMode.name());
            Logger.recordOutput("Turret/TrackAction", lastTrackAction);
            Logger.recordOutput("Turret/CurrentAngle", currentAngle);
            Logger.recordOutput("Turret/Setpoint", lastSetpointDegrees);
            Logger.recordOutput("Turret/CamYawInput", camAngleVal);
            Logger.recordOutput("Turret/AngVelLead", angVelLead);
            Logger.recordOutput("Turret/RobotOmegaDegPerSec", omegaVal);
            Logger.recordOutput("Turret/HubVisible", hubVisible.getAsBoolean());
            Logger.recordOutput("Turret/HubFresh", hubFresh.getAsBoolean());
            Logger.recordOutput("Turret/InShootingZone", inShootingZone.getAsBoolean());
            Logger.recordOutput("Turret/AutoTrackEnabled", autoTrackEnabled);
            Logger.recordOutput("Turret/JogInput", manualJogInput.getAsDouble());
            Logger.recordOutput("Turret/Debounce", trackingDebounceCount);
            Logger.recordOutput("Turret/BrakeFrames", cameraBrakeFrames);
            Logger.recordOutput("Turret/HoldAngle", holdAngleDegrees);

            // Console log (1 Hz) — human-readable summary for RioLog quick check
            double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (now - lastTrackLogTime >= 1.0) {
                lastTrackLogTime = now;
                System.out.printf(
                    "[Turret] mode=%s action=%s angle=%.1f setpt=%.1f camYaw=%.1f lead=%.1f omega=%.0f visible=%s fresh=%s inZone=%s autoTrack=%s debounce=%d brake=%d%n",
                    trackingMode, lastTrackAction,
                    currentAngle, lastSetpointDegrees, camAngleVal,
                    angVelLead, omegaVal,
                    hubVisible.getAsBoolean(), hubFresh.getAsBoolean(),
                    inShootingZone.getAsBoolean(), autoTrackEnabled,
                    trackingDebounceCount, cameraBrakeFrames);
            }
        }))
        .finallyDo(interrupted -> {
            setTurretAngle(getTurretAngleDegrees());
        })
        .withName("Auto Track Hub");
    }

    // ==================== Simple Tracker Command ====================

    /**
     * Simple proportional tracker — minimal complexity, one tuning knob (K).
     *
     * <p>When hub camera has a fresh frame: apply {@code camYaw * K} as percent
     * output. The turret moves proportionally to the offset and stops naturally
     * as it centers on the hub. When stale: output zero, planetary holds position.
     *
     * <p>When hub is not visible: use pose-based bearing to point the turret
     * toward the hub (one-shot Motion Magic), or slow sweep if no pose available.
     *
     * @param cameraYaw    turret-relative camera yaw offset (degrees)
     * @param hubVisible   sticky hub visibility
     * @param hubFresh     true only on frames with fresh camera data
     * @param hubPoseAngle pose-based bearing to hub (degrees, robot-relative)
     * @param hubPoseValid whether pose-based bearing is available
     * @param manualJogInput raw joystick axis [-1, 1] for manual override
     */
    public Command simpleTrackCommand(
            DoubleSupplier cameraYaw,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            DoubleSupplier hubPoseAngle,
            BooleanSupplier hubPoseValid,
            DoubleSupplier manualJogInput) {

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            scanDirection = 1;
        }, this)
        .andThen(run(() -> {
            if (handleManualJog(manualJogInput)) return;

            double currentAngle = getTurretAngleDegrees();

            if (hubVisible.getAsBoolean()) {
                // === HUB VISIBLE ===
                trackingMode = TrackingMode.CAMERA;

                if (hubFresh.getAsBoolean()) {
                    // Fresh frame: proportional percent output
                    double camYaw = cameraYaw.getAsDouble();
                    double output = camYaw * TurretTrackingConstants.kSimpleTrackK;
                    output = MathUtil.clamp(output,
                        -TurretTrackingConstants.kSimpleTrackMaxPercent,
                         TurretTrackingConstants.kSimpleTrackMaxPercent);
                    turretMotor.set(output);
                    holdAngleDegrees = currentAngle;
                    lastTrackAction = String.format("TRACK %.1f%% yaw=%.1f", output * 100, camYaw);
                } else {
                    // Stale frame: stop motor, planetary holds position
                    turretMotor.set(0);
                    lastTrackAction = "HOLD";
                }
            } else {
                // === HUB NOT VISIBLE ===
                if (hubPoseValid.getAsBoolean()) {
                    // Pose available: point turret toward hub bearing.
                    // poseAngle is robot-relative (0°=front, ±180°=back).
                    // Camera/shooter faces backward, so hub at 180° = turret forward.
                    trackingMode = TrackingMode.CAMERA;
                    double bearing = hubPoseAngle.getAsDouble();
                    double turretTarget = poseAngleToTurretAngle(bearing);
                    turretTarget = MathUtil.clamp(turretTarget, 0, TurretConstants.kTurretMaxDegrees);
                    setTurretAngle(turretTarget);
                    lastTrackAction = String.format("POSE aim=%.1f", turretTarget);
                } else {
                    // No pose: slow sweep
                    trackingMode = TrackingMode.SWEEP;
                    if (currentAngle >= TurretConstants.kTurretMaxDegrees
                            - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                    else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                    turretMotor.set(scanDirection * TurretTrackingConstants.kSimpleTrackSweepPercent);
                    lastTrackAction = "SWEEP dir=" + scanDirection;
                }
            }

            // Logging
            Logger.recordOutput("Turret/TrackMode", trackingMode.name());
            Logger.recordOutput("Turret/TrackAction", lastTrackAction);
            Logger.recordOutput("Turret/CurrentAngle", currentAngle);
            Logger.recordOutput("Turret/HubVisible", hubVisible.getAsBoolean());
            Logger.recordOutput("Turret/HubFresh", hubFresh.getAsBoolean());
        }))
        .finallyDo(interrupted -> {
            turretMotor.set(0);
            holdAngleDegrees = getTurretAngleDegrees();
        })
        .withName("Simple Track");
    }

    // ==================== Pose-Lock Tracker Command ====================

    /**
     * Pose-lock tracker — uses odometry + gyro for smooth continuous tracking,
     * camera only needed to acquire and refine.
     *
     * <p><b>Phase 1 (Acquire):</b> Use pose-based bearing to point turret at
     * the hub. If pose is unavailable, slow sweep. Once camera sees the hub,
     * record the world-frame hub position and transition to Phase 2.
     *
     * <p><b>Phase 2 (Lock):</b> Continuously compute turret angle from
     * {@code atan2(hubY - robotY, hubX - robotX) - robotHeading}. This runs
     * at the control loop rate (50 Hz from gyro/odometry), not the camera
     * rate. Result: smooth, continuous motion with no stepping or jitter.
     *
     * <p><b>Refine:</b> When fresh camera frames arrive, use the small camYaw
     * offset to correct drift in the locked hub position.
     *
     * @param cameraYaw      turret-relative camera yaw offset (degrees)
     * @param hubVisible     sticky hub visibility
     * @param hubFresh       true only on frames with fresh camera data
     * @param poseSupplier   robot pose (for bearing computation)
     * @param hubCenter      hub center position on the field
     * @param manualJogInput raw joystick axis [-1, 1] for manual override
     */
    public Command poseTrackCommand(
            DoubleSupplier cameraYaw,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Translation2d> hubCenter,
            DoubleSupplier manualJogInput) {

        // Mutable state for the lock phase
        boolean[] locked = { false };

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            scanDirection = 1;
            locked[0] = false;
        }, this)
        .andThen(run(() -> {
            if (handleManualJog(manualJogInput)) return;

            double currentAngle = getTurretAngleDegrees();
            var pose = poseSupplier.get();
            var hub = hubCenter.get();

            // Compute pose-based turret angle (runs every cycle at 50 Hz)
            double dx = hub.getX() - pose.getX();
            double dy = hub.getY() - pose.getY();
            double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
            double robotHeading = pose.getRotation().getDegrees();
            double robotRelative = worldBearing - robotHeading;
            double poseTurretAngle = poseAngleToTurretAngle(robotRelative);

            if (hubVisible.getAsBoolean() && hubFresh.getAsBoolean()) {
                // Camera sees hub with fresh data — we can lock or refine
                locked[0] = true;
                trackingMode = TrackingMode.CAMERA;

                // Refine: use camYaw to correct the pose-based angle.
                // Small camYaw means pose is accurate; large means drift.
                // Blend: 80% pose, 20% camera correction for stability.
                double camYaw = cameraYaw.getAsDouble();
                double correctedAngle = poseTurretAngle + camYaw * 0.2;
                correctedAngle = MathUtil.clamp(correctedAngle, 0, TurretConstants.kTurretMaxDegrees);
                setTurretAngle(correctedAngle);
                lastTrackAction = String.format("LOCK+CAM %.1f° yaw=%.1f", correctedAngle, camYaw);

            } else if (locked[0]) {
                // Locked but no fresh camera — use pure pose (smooth, 50 Hz)
                trackingMode = TrackingMode.CAMERA;
                double clampedAngle = MathUtil.clamp(poseTurretAngle, 0, TurretConstants.kTurretMaxDegrees);

                // If pose points outside turret range, we've lost the lock
                if (poseTurretAngle < -10 || poseTurretAngle > TurretConstants.kTurretMaxDegrees + 10) {
                    locked[0] = false;
                    lastTrackAction = "LOCK LOST (out of range)";
                } else {
                    setTurretAngle(clampedAngle);
                    lastTrackAction = String.format("LOCK %.1f°", clampedAngle);
                }

            } else if (hubVisible.getAsBoolean()) {
                // Hub visible but stale — use pose to aim, wait for fresh frame to lock
                trackingMode = TrackingMode.CAMERA;
                double clampedAngle = MathUtil.clamp(poseTurretAngle, 0, TurretConstants.kTurretMaxDegrees);
                setTurretAngle(clampedAngle);
                lastTrackAction = String.format("POSE AIM %.1f°", clampedAngle);

            } else {
                // Hub not visible, not locked — acquire
                locked[0] = false;

                // Try pose-based pointing if pose is valid (not at origin)
                if (pose.getTranslation().getNorm() > 1.0) {
                    trackingMode = TrackingMode.CAMERA;
                    double clampedAngle = MathUtil.clamp(poseTurretAngle, 0, TurretConstants.kTurretMaxDegrees);
                    setTurretAngle(clampedAngle);
                    lastTrackAction = String.format("ACQUIRE pose=%.1f°", clampedAngle);
                } else {
                    // No valid pose — slow sweep
                    trackingMode = TrackingMode.SWEEP;
                    if (currentAngle >= TurretConstants.kTurretMaxDegrees
                            - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                    else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                    turretMotor.set(scanDirection * TurretTrackingConstants.kSimpleTrackSweepPercent);
                    lastTrackAction = "SWEEP dir=" + scanDirection;
                }
            }

            // Logging
            Logger.recordOutput("Turret/TrackMode", trackingMode.name());
            Logger.recordOutput("Turret/TrackAction", lastTrackAction);
            Logger.recordOutput("Turret/CurrentAngle", currentAngle);
            Logger.recordOutput("Turret/PoseTurretAngle", poseTurretAngle);
            Logger.recordOutput("Turret/Locked", locked[0]);
            Logger.recordOutput("Turret/HubVisible", hubVisible.getAsBoolean());
            Logger.recordOutput("Turret/HubFresh", hubFresh.getAsBoolean());
        }))
        .finallyDo(interrupted -> {
            turretMotor.set(0);
            holdAngleDegrees = getTurretAngleDegrees();
        })
        .withName("Pose Track");
    }

    // ==================== Manual Align Command ====================

    /**
     * One-shot manual align: sweeps until the camera finds the hub, then locks on.
     * No auto-track toggle or shooting zone checks — drive team presses button, turret finds hub.
     * Holds locked position when the command ends (button release or cancel).
     *
     * @param cameraAngle  turret-relative yaw offset (degrees)
     * @param hubVisible   sticky visibility from vision layer
     * @param hubFresh     raw per-frame detection
     */
    public Command manualAlignCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh) {

        return Commands.runOnce(() -> {
            scanDirection = 1;
        }, this)
        .andThen(run(() -> {
            double currentAngle = getTurretAngleDegrees();

            if (hubVisible.getAsBoolean()) {
                trackingMode = TrackingMode.CAMERA;
                // Remember which side the hub is for faster reacquisition
                scanDirection = (cameraAngle.getAsDouble() >= 0) ? 1 : -1;
                double rawTarget = currentAngle + cameraAngle.getAsDouble();
                if (rawTarget < 0 || rawTarget > TurretConstants.kTurretMaxDegrees) {
                    // Hub is past mechanical limits — hold at nearest limit
                    setTurretAngle(MathUtil.clamp(rawTarget, 0, TurretConstants.kTurretMaxDegrees));
                } else if (hubFresh.getAsBoolean()) {
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * cameraTrackingGain;
                    setTurretAngle(smoothed);
                } else {
                    setTurretAngle(lastSetpointDegrees);
                }
            } else {
                // SWEEP: continuous smooth scan, reverse at limits
                trackingMode = TrackingMode.SWEEP;
                if (currentAngle >= TurretConstants.kTurretMaxDegrees - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                setTurretAngle(currentAngle + scanDirection * TurretTrackingConstants.kScanStepDeg);
            }
        }))
        .finallyDo(interrupted -> {
            holdAngleDegrees = getTurretAngleDegrees();
            setTurretAngle(holdAngleDegrees);
        })
        .withName("Manual Align");
    }

}
