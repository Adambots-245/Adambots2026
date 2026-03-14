package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import static edu.wpi.first.units.Units.Degrees;

import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.sensors.BaseAbsoluteEncoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Turret subsystem with position-controlled PID via onboard motor controller.
 * Uses a 10-turn potentiometer for absolute position sensing — no calibration needed.
 * The pot seeds the motor encoder on construction and re-syncs periodically.
 */
@Logged
public class TurretSubsystem extends SubsystemBase {

    /** Tracking state for telemetry. */
    enum TrackingMode { HOLD, CAMERA, POSE, SWEEP }

    private final BaseMotor turretMotor;
    private final BaseAbsoluteEncoder turretPot;

    private double trackingToleranceDeg = TurretTrackingConstants.kTrackingToleranceDeg;

    // Track last setpoint for isAtTarget()
    private double lastSetpointDegrees = 0;

    // Potentiometer calibration: pot reading at turret 0° and 180° (tunable via dashboard)
    private double potAtZeroDeg = TurretConstants.kTurretPotAtZeroDeg;
    private double potAtMaxDeg = TurretConstants.kTurretPotAtMaxDeg;

    // Auto-track toggle (driver opts in via Button 5)
    private boolean autoTrackEnabled = false;
    private boolean wasAutoTracking = false;
    private double holdAngleDegrees = 0.0;

    // Current tracking mode for telemetry
    private TrackingMode trackingMode = TrackingMode.HOLD;

    // Camera hysteresis: require N consecutive valid frames before switching to camera tier
    private int camValidFrames = 0;

    // Pose-fallback timer: tracks how long camera has been lost while in pose mode
    private double cameraLostTimestamp = 0.0;
    private boolean cameraWasTracking = false;

    // Sweep bounce direction (true = sweeping toward +amplitude)
    private boolean sweepGoingRight = true;

    // Pose-to-turret offset: poseAngle - offset = turret angle.
    // Derived from kTurretForwardDegrees (the turret angle that faces robot forward).
    private double poseOffsetDegrees = 360.0 - TurretConstants.kTurretForwardDegrees;

    public TurretSubsystem(BaseMotor turretMotor, BaseAbsoluteEncoder turretPot) {
        this.turretMotor = turretMotor;
        this.turretPot = turretPot;
        configureMotors();

        // Seed motor encoder from potentiometer absolute position
        double absoluteDeg = getPotAngleDegrees();
        double rotations = (absoluteDeg / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.setPosition(rotations);
    }

    private void configureMotors() {
        turretMotor.configure()
            .pid(TurretConstants.kTurretP, TurretConstants.kTurretI,
                 TurretConstants.kTurretD, TurretConstants.kTurretFF)
            .brakeMode(true)
            .currentLimits(TurretConstants.kTurretStallCurrentLimit,
                           TurretConstants.kTurretFreeCurrentLimit, 3000)
            .motionMagic(
                RotationsPerSecond.of(TurretConstants.kTurretCruiseVelocity),
                RotationsPerSecondPerSecond.of(TurretConstants.kTurretAcceleration),
                TurretConstants.kTurretJerk)
            .apply();
    }

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setTurretPID(double p, double i, double d, double ff) {
        turretMotor.setPID(0, p, i, d, ff);
    }

    public void setTrackingTolerance(double deg) {
        trackingToleranceDeg = deg;
    }

    public void setPoseOffset(double deg) {
        poseOffsetDegrees = deg;
    }

    public void setPotAtZeroDeg(double deg) {
        potAtZeroDeg = deg;
    }

    public void setPotAtMaxDeg(double deg) {
        potAtMaxDeg = deg;
    }

    // ==================== Turret Control ====================

    public void setTurretAngle(double degrees) {
        degrees = MathUtil.clamp(degrees, 0, TurretConstants.kTurretMaxDegrees);
        lastSetpointDegrees = degrees;
        double rotations = (degrees / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.set(ControlMode.MOTION_MAGIC, rotations);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public double getTurretAngleDegrees() {
        return (turretMotor.getPosition() / TurretConstants.kTurretGearRatio) * 360.0;
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

        // Dashboard telemetry (read angle once to avoid redundant CAN bus reads)
        double currentAngle = getTurretAngleDegrees();
        SmartDashboard.putNumber("Turret/Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Turret/Pot Raw (deg)", getRawPotDegrees());
        SmartDashboard.putNumber("Turret/Pot Turret (deg)", getPotAngleDegrees());
        SmartDashboard.putBoolean("Turret/AutoTrack", autoTrackEnabled);
        SmartDashboard.putNumber("Turret/Setpoint (deg)", lastSetpointDegrees);
        SmartDashboard.putNumber("Turret/Error (deg)", lastSetpointDegrees - currentAngle);
        SmartDashboard.putString("Turret/TrackingMode", trackingMode.name());
    }

    // ==================== Command Factories ====================

    /** Continuously aims turret at angle from supplier (for vision tracking). */
    public Command aimTurretCommand(DoubleSupplier angleSupplier) {
        return run(() -> setTurretAngle(angleSupplier.getAsDouble()))
            .withName("Aim Turret Dynamic");
    }

    /** Advances turret position via Motion Magic each cycle. Holds final position on release. */
    public Command scanCommand(double speed) {
        return run(() -> {
            double current = getTurretAngleDegrees();
            double step = speed * TurretConstants.kTurretManualStepDeg;
            setTurretAngle(current + step);
        }).finallyDo(interrupted -> {
            holdAngleDegrees = getTurretAngleDegrees();
        }).withName("Scan Turret");
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

    // ==================== Vision Tracking Commands ====================

    private double poseAngleToTurretAngle(double poseAngle) {
        return TurretTracking.poseAngleToTurretAngle(poseAngle, poseOffsetDegrees);
    }

    /**
     * Unified auto-track command: keeps the turret aimed at the hub using the best
     * available sensor data, with lead angle compensation for shooting while moving.
     *
     * <h3>How it works (priority order):</h3>
     * <ol>
     *   <li><b>CAMERA</b> — Camera sees the hub? Aim directly using the camera's
     *       reported offset from turret center. This is the most accurate source
     *       because it measures the actual hub position, not an estimate.</li>
     *   <li><b>POSE</b> — Camera lost but odometry knows where we are? Compute
     *       the bearing to the hub from our pose and convert it to a turret angle.
     *       Less accurate (odometry drifts), but keeps us close.</li>
     *   <li><b>SWEEP</b> — Camera has been lost for more than 2 seconds and we're
     *       still on pose? The pose estimate might be wrong, so oscillate ±15°
     *       around the pose target to help the camera re-acquire the hub.</li>
     *   <li><b>HOLD</b> — Nothing available (no camera, no pose), or the driver
     *       has disabled auto-tracking (Button 5). Just hold the current angle.</li>
     * </ol>
     *
     * <h3>Lead angle compensation (shooting while moving):</h3>
     * When the robot is driving, the ball inherits the robot's sideways velocity.
     * Without compensation, a ball aimed directly at the hub will miss because it
     * drifts during flight. We compute a small angular offset that aims the turret
     * slightly "into" the robot's motion to cancel this drift.
     * See {@link TurretTracking#computeLeadAngleDeg} for the math.
     *
     * <h3>Camera hysteresis:</h3>
     * The camera must report a valid target for 3 consecutive frames (60ms) before
     * we trust it. This prevents single-frame false positives from jerking the turret.
     *
     * <h3>All moves use Motion Magic</h3>
     * Every setTurretAngle() call goes through the motor controller's Motion Magic
     * profile (trapezoidal velocity). This keeps motion smooth even when switching
     * between tracking modes — no sudden jerks or oscillations.
     *
     * @param cameraAngle    turret-relative offset angle from the shooter camera (degrees).
     *                       Positive = hub is to the right of turret center.
     * @param cameraHasTarget whether the shooter camera currently sees the hub
     * @param poseAngle      robot-relative bearing to the hub from pose estimation (degrees)
     * @param poseHasTarget  whether pose estimation has a valid hub bearing
     * @param robotHeadingRad robot heading in field frame (radians, CCW positive) for lead angle
     * @param fieldVxMps     robot X velocity in field frame (m/s) for lead angle
     * @param fieldVyMps     robot Y velocity in field frame (m/s) for lead angle
     * @param distanceM      distance to hub (meters) for lead angle
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle,
            BooleanSupplier poseHasTarget,
            DoubleSupplier robotHeadingRad,
            DoubleSupplier fieldVxMps,
            DoubleSupplier fieldVyMps,
            DoubleSupplier distanceM) {

        // ---------------------------------------------------------------
        // Command structure: runOnce(init) → run(execute) → finallyDo(end)
        //
        // We use Commands.runOnce().andThen(run()) instead of a boolean[]
        // flag because Java lambdas can only capture effectively-final
        // local variables. This pattern gives us a clean "initialize once"
        // step without needing a mutable boolean hack.
        // ---------------------------------------------------------------

        return Commands.runOnce(() -> {
            // === INITIALIZATION (runs once when command starts) ===
            // Capture the turret's current position as the "hold" angle.
            // If auto-track is disabled, we'll hold here until the driver
            // re-enables it.
            holdAngleDegrees = getTurretAngleDegrees();
            wasAutoTracking = false;
            camValidFrames = 0;
            cameraWasTracking = false;
            cameraLostTimestamp = 0.0;
            sweepGoingRight = true;
        }, this)
        .andThen(run(() -> {
            // === EXECUTE (runs every 20ms cycle) ===

            // ------ Auto-track toggle (Button 5) ------
            // If the driver has disabled auto-tracking, just hold position.
            // When they disable it, we capture the current angle so the turret
            // doesn't snap to some stale target when re-enabled.
            if (!autoTrackEnabled) {
                if (wasAutoTracking) {
                    // Driver just toggled off — freeze at current position
                    holdAngleDegrees = getTurretAngleDegrees();
                    wasAutoTracking = false;
                }
                trackingMode = TrackingMode.HOLD;
                setTurretAngle(holdAngleDegrees);
                return;
            }
            wasAutoTracking = true;

            // ------ Camera hysteresis ------
            // Count consecutive frames where the camera reports a valid target.
            // Only trust the camera after kCamHysteresisFrames (3) in a row.
            // This prevents single-frame false positives from causing jitter.
            boolean camValid = cameraHasTarget.getAsBoolean();
            camValidFrames = camValid ? camValidFrames + 1 : 0;
            boolean useCam = camValid && camValidFrames >= TurretTrackingConstants.kCamHysteresisFrames;
            boolean poseValid = poseHasTarget.getAsBoolean();

            // ------ Lead angle compensation ------
            // Compute how many degrees to offset the turret aim to account for
            // the robot's lateral motion. This is applied to ALL tracking modes
            // (camera, pose, sweep) because the ball always inherits the robot's
            // velocity regardless of how we determined the aim direction.
            double currentAngle = getTurretAngleDegrees();
            double leadAngle = TurretTracking.computeLeadAngleDeg(
                currentAngle,
                robotHeadingRad.getAsDouble(),
                fieldVxMps.getAsDouble(),
                fieldVyMps.getAsDouble(),
                distanceM.getAsDouble(),
                TurretTrackingConstants.kBallExitSpeedMps);

            if (useCam) {
                // ------ CAMERA MODE (highest priority) ------
                // The camera directly measures the hub's position relative to the
                // turret. We add the camera offset to our current angle to get the
                // absolute target. This is the most accurate tracking mode.
                trackingMode = TrackingMode.CAMERA;
                double target = MathUtil.clamp(
                    currentAngle + cameraAngle.getAsDouble() + leadAngle,
                    0, TurretConstants.kTurretMaxDegrees);
                setTurretAngle(target);

                // Reset the pose-fallback timer because the camera is healthy.
                // If we lose the camera later, the timer starts from zero.
                cameraWasTracking = true;
                cameraLostTimestamp = 0.0;

            } else if (poseValid) {
                // ------ POSE / SWEEP MODE (camera lost, odometry available) ------
                // Compute where the hub should be based on our odometry pose.
                // poseAngleToTurretAngle() converts the robot-relative bearing
                // into turret coordinates.
                double poseTarget = MathUtil.clamp(
                    poseAngleToTurretAngle(poseAngle.getAsDouble()) + leadAngle,
                    0, TurretConstants.kTurretMaxDegrees);

                // Track how long the camera has been lost. If the camera was
                // working and just went away, record the timestamp.
                double now = Timer.getFPGATimestamp();
                if (cameraWasTracking) {
                    cameraLostTimestamp = now;
                    cameraWasTracking = false;
                }

                double cameraLostDuration = (cameraLostTimestamp > 0)
                    ? (now - cameraLostTimestamp) : 0.0;

                if (cameraLostDuration > TurretTrackingConstants.kPoseFallbackSweepTimeSec) {
                    // --- SWEEP: camera lost for too long, help it re-acquire ---
                    // Bounce between two Motion Magic endpoints around the pose
                    // target. Each endpoint gets a full trapezoidal profile, so
                    // the motor controller handles all the acceleration smoothly.
                    trackingMode = TrackingMode.SWEEP;
                    double amplitude = TurretTrackingConstants.kPoseFallbackSweepAmplitudeDeg;
                    double sweepTarget = MathUtil.clamp(
                        poseTarget + (sweepGoingRight ? amplitude : -amplitude),
                        0, TurretConstants.kTurretMaxDegrees);
                    if (Math.abs(currentAngle - sweepTarget)
                            < TurretTrackingConstants.kSweepArrivalToleranceDeg) {
                        sweepGoingRight = !sweepGoingRight;
                        sweepTarget = MathUtil.clamp(
                            poseTarget + (sweepGoingRight ? amplitude : -amplitude),
                            0, TurretConstants.kTurretMaxDegrees);
                    }
                    setTurretAngle(sweepTarget);
                } else {
                    // --- POSE: camera recently lost, trust odometry ---
                    // The pose estimate is usually still accurate shortly after
                    // losing the camera. Aim directly at the computed target
                    // and wait for the camera to re-acquire.
                    trackingMode = TrackingMode.POSE;
                    setTurretAngle(poseTarget);
                }

            } else {
                // ------ HOLD MODE (nothing available) ------
                // No camera, no pose — we have no idea where the hub is.
                // Hold the turret at its current position to avoid random motion.
                // The driver can use D-pad manual scan to find the hub.
                trackingMode = TrackingMode.HOLD;
                setTurretAngle(currentAngle);
            }
        }))
        .finallyDo(interrupted -> {
            // === CLEANUP (runs when command ends or is interrupted) ===
            // Hold at current position rather than stopTurret() (duty cycle 0).
            // This keeps Motion Magic PID active so the turret doesn't coast/drift
            // during the ~20ms gap before the default command restarts.
            // When used inside autoShootCommand (not as default), this also prevents
            // the turret from going limp between sequence steps.
            setTurretAngle(getTurretAngleDegrees());
        })
        .withName("Auto Track Hub");
    }

    // ==================== Diagnostic Command ====================

    // Stepped diagnostic state (persists across button presses)
    private int diagStep = 0;
    private final StringBuilder diagLog = new StringBuilder();
    private String diagInstruction = "Press 'Turret Diag' to start";

    /** Current diagnostic step instruction (for dashboard string widget). */
    public String getDiagInstruction() { return diagInstruction; }

    /**
     * Creates a stepped diagnostic command. Each dashboard button press advances
     * one step. Steps 1-3 jog the turret to known angles so you can observe where
     * it physically points. Steps 4-5 capture pose/bearing snapshots at positions
     * you choose. Step 6 captures lead angle data while the robot is moving, then
     * prints ALL collected results to the console and resets.
     *
     * <h3>Steps:</h3>
     * <ol>
     *   <li>Jog turret to 0° — observe where the shooter physically points</li>
     *   <li>Jog turret to 90° — observe where the shooter physically points</li>
     *   <li>Jog turret to 180° — observe where the shooter physically points</li>
     *   <li>You position robot so you know where the hub is relative to it. Press button — captures bearing + turretFromPose</li>
     *   <li>You rotate/reposition robot to a different known orientation. Press button — captures second snapshot</li>
     *   <li>You strafe the robot sideways. Press button while moving — captures velocity + lead angle, then PRINTS ALL RESULTS and resets</li>
     * </ol>
     *
     * <p>Paste the full output for review. Describe what you physically observed
     * at each step (where was the turret pointing? where was the hub relative to the robot?).</p>
     */
    public Command turretDiagnosticCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle,
            BooleanSupplier poseHasTarget,
            DoubleSupplier robotHeadingRad,
            DoubleSupplier fieldVxMps,
            DoubleSupplier fieldVyMps,
            DoubleSupplier distanceM,
            Supplier<String> poseStringSupplier) {

        return Commands.runOnce(() -> {
            diagStep++;

            switch (diagStep) {
                case 1: // Jog turret to 0°
                    diagLog.setLength(0); // reset from any previous run
                    setTurretAngle(0);
                    diagLog.append("Step 1: Turret jogged to 0 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagInstruction = "1/6: Turret -> 0 deg. Note where shooter points. Press again.";
                    break;

                case 2: // Jog turret to 90°
                    diagLog.append(String.format("  Encoder settled: %.1f deg\n\n", getTurretAngleDegrees()));
                    setTurretAngle(90);
                    diagLog.append("Step 2: Turret jogged to 90 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagInstruction = "2/6: Turret -> 90 deg. Note where shooter points. Press again.";
                    break;

                case 3: // Jog turret to 180°
                    diagLog.append(String.format("  Encoder settled: %.1f deg\n\n", getTurretAngleDegrees()));
                    setTurretAngle(180);
                    diagLog.append("Step 3: Turret jogged to 180 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagInstruction = "3/6: Turret -> 180 deg. Observe shooter direction. "
                        + "Then: point INTAKE at hub. Press when stopped.";
                    break;

                case 4: // Pose snapshot #1 — intake facing hub
                    diagLog.append(String.format("  Encoder settled: %.1f deg\n\n", getTurretAngleDegrees()));
                    setTurretAngle(90); // jog turret to forward so camera faces same way as intake
                    appendPoseSnapshot(diagLog, "Step 4: Intake facing hub (turret reset to 90)",
                        poseAngle, poseHasTarget, cameraAngle, cameraHasTarget,
                        robotHeadingRad, distanceM, poseStringSupplier);
                    diagLog.append("  EXPECTED: bearing ~ 0 deg, turretFromPose ~ 90 deg\n\n");
                    diagInstruction = "4/6: Captured. Now rotate robot 90 deg CW (viewed from above). "
                        + "Press when stopped.";
                    break;

                case 5: // Pose snapshot #2 — rotated 90° CW from step 4
                    appendPoseSnapshot(diagLog, "Step 5: After 90 deg CW rotation (hub now to robot left)",
                        poseAngle, poseHasTarget, cameraAngle, cameraHasTarget,
                        robotHeadingRad, distanceM, poseStringSupplier);
                    diagLog.append("  EXPECTED: bearing ~ +90 deg (CCW+), turretFromPose ~ 180 deg\n\n");
                    diagInstruction = "5/6: Captured. Point intake at hub again. "
                        + "STRAFE RIGHT with joystick. Press WHILE MOVING.";
                    break;

                case 6: // Lead angle snapshot — strafing right with intake facing hub
                    appendLeadSnapshot(diagLog, "Step 6: Strafing RIGHT with intake facing hub",
                        cameraAngle, cameraHasTarget, poseAngle, poseHasTarget,
                        robotHeadingRad, fieldVxMps, fieldVyMps, distanceM, poseStringSupplier);
                    diagLog.append("  EXPECTED: hub is ahead, strafing right = rightward cross-track\n");
                    diagLog.append("            cross-track < 0, lead angle > 0 (aim left to compensate)\n\n");

                    // Print full results
                    StringBuilder out = new StringBuilder();
                    out.append("\n============ TURRET DIAGNOSTIC FULL RESULTS ============\n");
                    out.append("Paste this output for review.\n");
                    out.append("========================================================\n\n");
                    out.append(diagLog);
                    out.append("=============== EXPECTED vs ACTUAL SUMMARY =============\n");
                    out.append("Step 1: Turret at 0 deg  = reverse hardware limit\n");
                    out.append("Step 2: Turret at 90 deg = should point same direction as intake (forward)\n");
                    out.append("Step 3: Turret at 180 deg = forward hardware limit\n");
                    out.append("Step 4: Intake facing hub → bearing ~ 0, turretFromPose ~ 90\n");
                    out.append("Step 5: 90 CW from step 4 → bearing ~ +90, turretFromPose ~ 180\n");
                    out.append("Step 6: Strafe right, intake at hub → cross-track < 0, lead > 0\n");
                    out.append("========================================================\n");

                    System.out.println(out.toString());

                    // Reset for next run
                    diagStep = 0;
                    diagLog.setLength(0);
                    diagInstruction = "Done! Paste console output. Press to restart.";
                    return;

                default:
                    diagStep = 0;
                    diagInstruction = "Press 'Turret Diag' to start";
                    return;
            }
        }).withName("Turret Diagnostic");
    }

    /** Appends a pose/bearing snapshot to the diagnostic log. */
    private void appendPoseSnapshot(StringBuilder log, String label,
            DoubleSupplier poseAngle, BooleanSupplier poseHasTarget,
            DoubleSupplier cameraAngle, BooleanSupplier cameraHasTarget,
            DoubleSupplier robotHeadingRad, DoubleSupplier distanceM,
            Supplier<String> poseStringSupplier) {
        double turretAngle = getTurretAngleDegrees();
        double bearing = poseAngle.getAsDouble();
        boolean poseVis = poseHasTarget.getAsBoolean();
        double turretFromPose = poseVis
            ? MathUtil.clamp(poseAngleToTurretAngle(bearing), 0, TurretConstants.kTurretMaxDegrees)
            : Double.NaN;

        log.append(label).append("\n");
        log.append(String.format("  Robot Pose:        %s\n", poseStringSupplier.get()));
        log.append(String.format("  Robot Heading:     %.1f deg (field, CCW+)\n",
            Math.toDegrees(robotHeadingRad.getAsDouble())));
        log.append(String.format("  Turret Encoder:    %.1f deg\n", turretAngle));
        log.append(String.format("  Pose Has Target:   %s\n", poseVis));
        log.append(String.format("  Hub Pose Bearing:  %.1f deg (robot-relative, CCW+)\n", bearing));
        log.append(String.format("  Turret from Pose:  %.1f deg (after conversion + clamp)\n", turretFromPose));
        log.append(String.format("  Camera Has Target: %s\n", cameraHasTarget.getAsBoolean()));
        log.append(String.format("  Camera Yaw:        %.1f deg (offset from turret center)\n",
            cameraAngle.getAsDouble()));
        log.append(String.format("  Hub Distance:      %.2f m\n", distanceM.getAsDouble()));
        log.append("\n");
    }

    /** Appends a lead-angle snapshot (while robot is moving) to the diagnostic log. */
    private void appendLeadSnapshot(StringBuilder log, String label,
            DoubleSupplier cameraAngle, BooleanSupplier cameraHasTarget,
            DoubleSupplier poseAngle, BooleanSupplier poseHasTarget,
            DoubleSupplier robotHeadingRad,
            DoubleSupplier fieldVxMps, DoubleSupplier fieldVyMps,
            DoubleSupplier distanceM, Supplier<String> poseStringSupplier) {
        double turretAngle = getTurretAngleDegrees();
        double heading = Math.toDegrees(robotHeadingRad.getAsDouble());
        double vx = fieldVxMps.getAsDouble();
        double vy = fieldVyMps.getAsDouble();
        double dist = distanceM.getAsDouble();

        double fieldAimDeg = heading + (turretAngle - TurretConstants.kTurretForwardDegrees);

        double leadAngle = TurretTracking.computeLeadAngleDeg(
            turretAngle, robotHeadingRad.getAsDouble(), vx, vy, dist,
            TurretTrackingConstants.kBallExitSpeedMps);

        double aimFieldRad = robotHeadingRad.getAsDouble()
            + Math.toRadians(turretAngle - TurretConstants.kTurretForwardDegrees);
        double crossTrackVel = -vx * Math.sin(aimFieldRad) + vy * Math.cos(aimFieldRad);

        log.append(label).append("\n");
        log.append(String.format("  Robot Pose:        %s\n", poseStringSupplier.get()));
        log.append(String.format("  Robot Heading:     %.1f deg (field, CCW+)\n", heading));
        log.append(String.format("  Turret Encoder:    %.1f deg\n", turretAngle));
        log.append(String.format("  Field Aim Dir:     %.1f deg (heading + turret - 90)\n", fieldAimDeg));
        log.append(String.format("  Hub Distance:      %.2f m\n", dist));
        log.append(String.format("  Robot Velocity:    vx=%.2f  vy=%.2f m/s (field)\n", vx, vy));
        log.append(String.format("  Cross-Track Vel:   %.2f m/s (perp to aim dir)\n", crossTrackVel));
        log.append(String.format("  Lead Angle:        %.2f deg (added to turret target)\n", leadAngle));
        log.append("\n");
    }
}
