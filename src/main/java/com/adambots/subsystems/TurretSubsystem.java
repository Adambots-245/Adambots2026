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

        // Dual PID slots for independent tuning:
        //   Slot 0 — Motion Magic (go-to-angle, hold, forward, sweep)
        //   Slot 1 — PositionVoltage (auto-track CAMERA, manual align)
        // Initialized with the same gains so current behavior is unchanged.
        // Tune slot 1 independently if tracking needs different kP/kD
        // (e.g., smoother convergence) vs go-to-angle (snappy profiled moves).
        //
        // kS is critical for turret: it adds a constant voltage in the direction
        // of error to overcome static friction from the 3D-printed gear mesh and
        // cable tray. Without it, the PID hovers at the friction breakaway
        // boundary and buzzes.
        // Dual PID slots (lib 2026.3.28 adds Slot1/Slot2 support for MinionMotor):
        //   Slot 0 — go-to-angle (setTurretAngle): lower kP for smooth moves
        //   Slot 1 — tracking (setTurretAngleTracking): higher kP for responsive tracking
        turretMotor.setPID(0,
                TurretConstants.kTurretP, TurretConstants.kTurretI,
                TurretConstants.kTurretD,
                TurretConstants.kTurretKV, TurretConstants.kTurretKS,
                TurretConstants.kTurretKA, TurretConstants.kTurretKG);
        turretMotor.setPID(1,
                TurretConstants.kTurretTrackingP, TurretConstants.kTurretI,
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
        turretMotor.setPID(0, p, i, d, kV, kS, kA, kG); // slot 0: go-to-angle
        // Slot 1: scale kP by the tracking/base ratio for responsive tracking
        double trackingP = p * (TurretConstants.kTurretTrackingP / TurretConstants.kTurretP);
        turretMotor.setPID(1, trackingP, i, d, kV, kS, kA, kG); // slot 1: tracking
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

    // ==================== Turret Control ====================

    /**
     * Move turret to a target angle using PositionVoltage with setpoint-
     * derivative velocity feedforward (slot 0, kP=18).
     *
     * <p>The velocity feedforward is computed from the position error:
     * {@code (setpoint - currentAngle) / kConvergeTimeSec}. This tells the
     * motor controller "move toward the target at this speed" via the kV
     * feedforward term, instead of relying solely on kP (which oscillates
     * through backlash). The velocity naturally decays to zero as the
     * turret approaches the setpoint — smooth deceleration without abrupt
     * stops or backlash fighting.
     *
     * <p>This is the same approach Team 5000 uses: PositionVoltage with
     * setpoint-derivative velocity on every call.
     */
    public void setTurretAngle(double degrees) {
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double posRot = (degrees / 360.0) * TurretConstants.kTurretMotorGearRatio;
        // Compute desired velocity: how fast to approach the setpoint.
        // Converge time of 0.2s means "close the gap in ~200ms".
        // The kV feedforward applies voltage proportional to this velocity,
        // giving the motor smooth speed guidance alongside position error.
        double errorDeg = degrees - getTurretAngleDegrees();
        double desiredVelDPS = errorDeg / TurretTrackingConstants.kConvergeTimeSec;
        desiredVelDPS = MathUtil.clamp(desiredVelDPS, -300, 300);
        double velRPS = (desiredVelDPS / 360.0) * TurretConstants.kTurretMotorGearRatio;
        turretMotor.setPositionWithVelocityFF(posRot, velRPS);
    }

    /**
     * Move turret to a target angle using PositionVoltage with velocity
     * feedforward. Use for continuous tracking (auto-track CAMERA mode).
     *
     * <p>Unlike {@link #setTurretAngle} (Motion Magic), this does NOT
     * generate a trapezoidal velocity profile. The PID acts directly on
     * position error, and the velocity feedforward term applies
     * {@code kV × velocity} as a constant voltage to anticipate motion.
     * This eliminates the trajectory-reset jitter that occurs when
     * Motion Magic receives a new target every 20 ms.
     *
     * <p>The velocity parameter is typically the robot's rotation
     * compensation: {@code -robotOmega} in turret deg/sec, so the motor
     * controller applies voltage to counter robot rotation without
     * displacing the position target.
     *
     * @param degrees      target turret angle in degrees
     * @param velDegPerSec expected turret angular velocity (deg/sec)
     */
    public void setTurretAngleTracking(double degrees, double velDegPerSec) {
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double posRot = (degrees / 360.0) * TurretConstants.kTurretMotorGearRatio;
        // Combine rotation compensation with setpoint-derivative velocity.
        // The setpoint-derivative gives smooth approach to the target;
        // the rotation comp counters robot rotation on top.
        double errorDeg = degrees - getTurretAngleDegrees();
        double approachVelDPS = errorDeg / TurretTrackingConstants.kConvergeTimeSec;
        approachVelDPS = MathUtil.clamp(approachVelDPS, -300, 300);
        double totalVelDPS = approachVelDPS + velDegPerSec;
        double velRPS = (totalVelDPS / 360.0) * TurretConstants.kTurretMotorGearRatio;
        turretMotor.setPositionWithVelocityFF(posRot, velRPS, 1);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    /**
     * Called by {@code RobotContainer.onDisabledInit()}. Replaces any latched
     * control request (PositionVoltage, Motion Magic, etc.) with
     * {@code DutyCycleOut(0)} so stale tracking commands don't resume driving
     * the turret on re-enable.
     */
    public void onDisable() {
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
            // Manual jog override: always available regardless of auto-track toggle.
            double jogRaw = manualJogInput.getAsDouble();
            double jogDeadbanded = MathUtil.applyDeadband(jogRaw, TurretConstants.kTurretJogDeadband);
            if (jogDeadbanded != 0.0) {
                double jogOutput = Math.copySign(jogDeadbanded * jogDeadbanded, jogDeadbanded)
                    * TurretConstants.kTurretJogMaxPercent;
                turretMotor.set(jogOutput);
                trackingMode = TrackingMode.JOG;
                lastTrackAction = String.format("JOG %.0f%%", jogOutput * 100);
                holdAngleDegrees = getTurretAngleDegrees();
                return;
            }
            if (trackingMode == TrackingMode.JOG) {
                holdAngleDegrees = getTurretAngleDegrees();
                lastSetpointDegrees = holdAngleDegrees;
                trackingMode = TrackingMode.HOLD;
            }

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

            // Robot rotation compensation via velocity feedforward.
            // WPILib convention: positive omega = CCW rotation.
            // To keep the turret stationary in world frame while the robot
            // rotates CCW (+omega), the turret must rotate CW relative to
            // robot = -omega in turret deg/sec. This is applied as velocity
            // feedforward to PositionVoltage — the motor controller applies
            // kV × velocity as a constant voltage that directly counters the
            // rotation, WITHOUT displacing the position target (unlike the
            // old angVelLead approach which added omega * leadTime to the
            // target position and caused snap-back when rotation stopped).
            double robotOmega = robotAngularVelDegPerSec.getAsDouble();
            double rotationCompVelDPS = -robotOmega;

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
                // Dead zone + debounce: hold steady unless offset exceeds tolerance for N frames
                if (Math.abs(camAngle) < trackingToleranceDeg) {
                    trackingDebounceCount = 0;
                    // DEADZONE: coast with position = current (error=0).
                    // kP and kS produce zero output. Only kV × velocity
                    // applies. Includes rotation compensation so the turret
                    // counter-rotates during DEADZONE instead of drifting.
                    double dzErrorDeg = lastSetpointDegrees - currentAngle;
                    double dzVelDPS = dzErrorDeg / TurretTrackingConstants.kConvergeTimeSec
                        + rotationCompVelDPS;
                    dzVelDPS = MathUtil.clamp(dzVelDPS, -300, 300);
                    double dzVelRPS = (dzVelDPS / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    double dzCurrentRot = (currentAngle / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    turretMotor.setPositionWithVelocityFF(dzCurrentRot, dzVelRPS);
                    lastTrackAction = "DEADZONE coast";
                } else if (++trackingDebounceCount < TurretTrackingConstants.kTrackingDebounceFrames) {
                    // Coast like DEADZONE — no kP/kS backlash fighting.
                    // Includes rotation compensation.
                    double dbErrorDeg = lastSetpointDegrees - currentAngle;
                    double dbVelDPS = dbErrorDeg / TurretTrackingConstants.kConvergeTimeSec
                        + rotationCompVelDPS;
                    dbVelDPS = MathUtil.clamp(dbVelDPS, -300, 300);
                    double dbVelRPS = (dbVelDPS / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    double dbCurrentRot = (currentAngle / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    turretMotor.setPositionWithVelocityFF(dbCurrentRot, dbVelRPS);
                    lastTrackAction = "DEBOUNCE " + trackingDebounceCount;
                } else if (rawTarget < 0 || rawTarget > TurretConstants.kTurretMaxDegrees) {
                    // Hub is past a mechanical stop. Use setTurretAngle (slot 0,
                    // kP=18) instead of setTurretAngleTracking (slot 1, kP=60) —
                    // gentler hold at the limit prevents the motor from pushing
                    // hard against the physical stop.
                    setTurretAngle(MathUtil.clamp(rawTarget, 0, TurretConstants.kTurretMaxDegrees));
                    lastTrackAction = "LIMIT clamp=" + String.format("%.1f", rawTarget);
                } else if (hubFresh.getAsBoolean()) {
                    // Fresh frame: compute new target from current angle + camera offset
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
                    // Decel zone: when the target is within 20° of either
                    // mechanical limit, use slot 0 (kP=18) for a gentler
                    // approach. Prevents the turret from slamming into stops
                    // at full kP=60 speed when tracking a hub near a limit.
                    double distToLimit = Math.min(smoothed,
                        TurretConstants.kTurretMaxDegrees - smoothed);
                    if (distToLimit < TurretTrackingConstants.kDecelZoneDeg) {
                        setTurretAngle(smoothed);
                        lastTrackAction = "TRACK decel";
                    } else {
                        setTurretAngleTracking(smoothed, rotationCompVelDPS);
                        lastTrackAction = "TRACK fresh";
                    }
                } else {
                    // Stale frame: coast toward setpoint using ONLY
                    // velocity feedforward. Position target is set to
                    // CURRENT angle (not setpoint), so posError = 0 →
                    // kP output = 0, kS output = 0. Only kV × velocity
                    // applies, providing a gentle, decaying push toward
                    // the setpoint without any kP/kS backlash fighting.
                    //
                    // This is why the "smooth" first deploy worked: the
                    // slot bug meant kP=0, and the turret coasted on
                    // kS alone. We're replicating that by zeroing the
                    // position error intentionally on stale frames.
                    double errorDeg = lastSetpointDegrees - currentAngle;
                    double coastVelDPS = errorDeg / TurretTrackingConstants.kConvergeTimeSec
                        + rotationCompVelDPS;
                    coastVelDPS = MathUtil.clamp(coastVelDPS, -300, 300);
                    double coastVelRPS = (coastVelDPS / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    // Position = current (error=0), velocity = approach
                    double currentRot = (currentAngle / 360.0) * TurretConstants.kTurretMotorGearRatio;
                    turretMotor.setPositionWithVelocityFF(currentRot, coastVelRPS);
                    lastTrackAction = "COAST stale";
                }
            } else {
                // SWEEP: continuous constant-speed scan, reverse at limits.
                // Uses percent output (not Motion Magic stepping) for smooth
                // motion — same reasoning as scanCommand for manual jog.
                // Soft limits provide firmware-level boundary protection.
                trackingMode = TrackingMode.SWEEP;
                if (sweepWarmupFrames > 0) {
                    sweepWarmupFrames--;
                    setTurretAngle(holdAngleDegrees);
                    lastTrackAction = "WARMUP " + sweepWarmupFrames;
                    return;
                }
                if (currentAngle >= TurretConstants.kTurretMaxDegrees - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                turretMotor.set(scanDirection * TurretTrackingConstants.kTurretSweepPercent);
                lastTrackAction = "SWEEP dir=" + scanDirection;
            }

            // ==================== Structured logging (AdvantageScope) ====================
            // Always active — writes to WPILog. Lets you see exactly what the
            // auto-track loop received and what it commanded, overlaid with
            // the Vision/ signals in AdvantageScope.
            double camAngleVal = cameraAngle.getAsDouble();
            Logger.recordOutput("Turret/TrackMode", trackingMode.name());
            Logger.recordOutput("Turret/TrackAction", lastTrackAction);
            Logger.recordOutput("Turret/CurrentAngle", currentAngle);
            Logger.recordOutput("Turret/Setpoint", lastSetpointDegrees);
            Logger.recordOutput("Turret/CamYawInput", camAngleVal);
            Logger.recordOutput("Turret/RobotOmegaDegPerSec", robotOmega);
            Logger.recordOutput("Turret/RotationCompVelDPS", rotationCompVelDPS);
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
                    "[Turret] mode=%s action=%s angle=%.1f setpt=%.1f camYaw=%.1f omega=%.0f rotComp=%.1f visible=%s fresh=%s inZone=%s autoTrack=%s debounce=%d brake=%d%n",
                    trackingMode, lastTrackAction,
                    currentAngle, lastSetpointDegrees, camAngleVal,
                    robotOmega, rotationCompVelDPS,
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

    // ==================== Manual Align Command ====================

    /**
     * One-shot manual align: sweeps until the camera finds the hub, then locks on.
     * No auto-track toggle or shooting zone checks — drive team presses button, turret finds hub.
     * Holds locked position when the command ends (button release or cancel).
     *
     * <p>Uses PositionVoltage with velocity feedforward for rotation compensation
     * when tracking (same approach as autoTrackCommand CAMERA mode), and percent
     * output for sweep (same approach as autoTrackCommand SWEEP mode).
     *
     * @param cameraAngle  turret-relative yaw offset (degrees)
     * @param hubVisible   sticky visibility from vision layer
     * @param hubFresh     raw per-frame detection
     * @param robotAngularVelDegPerSec  robot yaw rate for rotation compensation
     */
    public Command manualAlignCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            DoubleSupplier robotAngularVelDegPerSec) {

        return Commands.runOnce(() -> {
            scanDirection = 1;
        }, this)
        .andThen(run(() -> {
            double currentAngle = getTurretAngleDegrees();
            double rotationCompVelDPS = -robotAngularVelDegPerSec.getAsDouble();

            if (hubVisible.getAsBoolean()) {
                trackingMode = TrackingMode.CAMERA;
                // Remember which side the hub is for faster reacquisition
                scanDirection = (cameraAngle.getAsDouble() >= 0) ? 1 : -1;
                double rawTarget = currentAngle + cameraAngle.getAsDouble();
                if (rawTarget < 0 || rawTarget > TurretConstants.kTurretMaxDegrees) {
                    // Gentle hold at limit (slot 0, kP=18)
                    setTurretAngle(MathUtil.clamp(rawTarget, 0, TurretConstants.kTurretMaxDegrees));
                } else if (hubFresh.getAsBoolean()) {
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
                    setTurretAngleTracking(smoothed, rotationCompVelDPS);
                } else {
                    setTurretAngleTracking(lastSetpointDegrees, rotationCompVelDPS);
                }
            } else {
                // SWEEP: constant percent output, reverse at limits
                trackingMode = TrackingMode.SWEEP;
                if (currentAngle >= TurretConstants.kTurretMaxDegrees - TurretTrackingConstants.kScanMarginDeg) scanDirection = -1;
                else if (currentAngle <= TurretTrackingConstants.kScanMarginDeg) scanDirection = 1;
                turretMotor.set(scanDirection * TurretTrackingConstants.kTurretSweepPercent);
            }
        }))
        .finallyDo(interrupted -> {
            holdAngleDegrees = getTurretAngleDegrees();
            setTurretAngle(holdAngleDegrees);
        })
        .withName("Manual Align");
    }

}
