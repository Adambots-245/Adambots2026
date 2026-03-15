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

import edu.wpi.first.math.MathUtil;

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
    enum TrackingMode { HOLD, CAMERA, SWEEP }

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

    // Camera hysteresis: charge/decay counter. Charges per valid frame, drains -1 per lost frame.
    private static final int CAM_COUNTER_MAX = TurretTrackingConstants.kCamCounterMax;
    private int camCounter = 0;

    // Search state: step through discrete angles, dwell at each to let camera detect
    private double searchAngleDeg = 0;
    private int searchDwellFrames = 0;

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
        SmartDashboard.putNumber("Turret/CamCounter", camCounter);
        SmartDashboard.putNumber("Turret/SearchAngle", searchAngleDeg);
        SmartDashboard.putNumber("Turret/SearchDwell", searchDwellFrames);
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

    /**
     * Auto-track command: search-then-lock system.
     * <ol>
     *   <li><b>SWEEP</b> — Steps to discrete angles (0°, 30°, 60°, …), dwelling at each
     *       to give the camera time to detect hub tags.</li>
     *   <li><b>CAMERA</b> — Hub tags found → track using camera yaw offset, with
     *       charge/decay hysteresis to ride through brief dropouts.</li>
     *   <li><b>HOLD</b> — Auto-track disabled or not in shooting zone.</li>
     * </ol>
     */
    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget,
            BooleanSupplier inShootingZone) {

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            wasAutoTracking = false;
            camCounter = 0;
            searchAngleDeg = TurretConstants.kTurretForwardDegrees;
            searchDwellFrames = 0;
        }, this)
        .andThen(run(() -> {
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

            // Charge/decay hysteresis: +kCamChargeRate per valid frame, -1 per lost frame
            boolean camValid = cameraHasTarget.getAsBoolean();
            camCounter = camValid
                ? Math.min(camCounter + TurretTrackingConstants.kCamChargeRate, CAM_COUNTER_MAX)
                : Math.max(camCounter - 1, 0);

            boolean useCam = camCounter >= TurretTrackingConstants.kCamHysteresisFrames;
            double currentAngle = getTurretAngleDegrees();

            if (useCam) {
                // CAMERA MODE — locked on hub
                trackingMode = TrackingMode.CAMERA;
                searchDwellFrames = 0;
                // Keep search position near current angle for quick re-acquisition
                searchAngleDeg = Math.round(currentAngle / TurretTrackingConstants.kSearchStepDeg)
                    * TurretTrackingConstants.kSearchStepDeg;
                if (camValid) {
                    // Compute where the hub actually is, then smoothly move setpoint toward it
                    double fullTarget = MathUtil.clamp(
                        currentAngle + cameraAngle.getAsDouble(),
                        0, TurretConstants.kTurretMaxDegrees);
                    // Exponential smoothing: move setpoint 30% of remaining distance each cycle
                    double smoothed = lastSetpointDegrees
                        + (fullTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
                    setTurretAngle(smoothed);
                } else {
                    // Brief dropout — hold last setpoint while counter drains
                    setTurretAngle(lastSetpointDegrees);
                }
            } else if (camCounter > 0) {
                // NEAR-LOCK — tags recently seen but below threshold.
                // Hold last known position instead of snapping to search grid.
                trackingMode = TrackingMode.CAMERA;
                setTurretAngle(lastSetpointDegrees);
            } else {
                // SEARCH MODE — no recent detections, step through positions
                trackingMode = TrackingMode.SWEEP;
                if (isAtTarget(5.0)) {
                    searchDwellFrames++;
                    if (searchDwellFrames >= TurretTrackingConstants.kManualAlignDwellFrames) {
                        searchAngleDeg += TurretTrackingConstants.kSearchStepDeg;
                        if (searchAngleDeg > TurretConstants.kTurretMaxDegrees) {
                            searchAngleDeg = 0;
                        }
                        searchDwellFrames = 0;
                    }
                }
                setTurretAngle(searchAngleDeg);
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
     */
    public Command manualAlignCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget) {

        return Commands.runOnce(() -> {
            camCounter = 0;
            searchAngleDeg = TurretConstants.kTurretForwardDegrees; // start from forward — hub is most likely ahead
            searchDwellFrames = 0;
        }, this)
        .andThen(run(() -> {
            boolean camValid = cameraHasTarget.getAsBoolean();
            camCounter = camValid
                ? Math.min(camCounter + TurretTrackingConstants.kCamChargeRate, CAM_COUNTER_MAX)
                : Math.max(camCounter - 1, 0);

            boolean useCam = camCounter >= TurretTrackingConstants.kCamHysteresisFrames;
            double currentAngle = getTurretAngleDegrees();

            if (useCam) {
                trackingMode = TrackingMode.CAMERA;
                searchDwellFrames = 0;
                searchAngleDeg = Math.round(currentAngle / TurretTrackingConstants.kSearchStepDeg)
                    * TurretTrackingConstants.kSearchStepDeg;
                if (camValid) {
                    double fullTarget = MathUtil.clamp(
                        currentAngle + cameraAngle.getAsDouble(),
                        0, TurretConstants.kTurretMaxDegrees);
                    double smoothed = lastSetpointDegrees
                        + (fullTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
                    setTurretAngle(smoothed);
                } else {
                    setTurretAngle(lastSetpointDegrees);
                }
            } else if (camCounter > 0) {
                trackingMode = TrackingMode.CAMERA;
                setTurretAngle(lastSetpointDegrees);
            } else {
                trackingMode = TrackingMode.SWEEP;
                if (isAtTarget(5.0)) {
                    searchDwellFrames++;
                    if (searchDwellFrames >= TurretTrackingConstants.kManualAlignDwellFrames) {
                        searchAngleDeg += TurretTrackingConstants.kSearchStepDeg;
                        if (searchAngleDeg > TurretConstants.kTurretMaxDegrees) {
                            searchAngleDeg = 0;
                        }
                        searchDwellFrames = 0;
                    }
                }
                setTurretAngle(searchAngleDeg);
            }
        }))
        .finallyDo(interrupted -> {
            holdAngleDegrees = getTurretAngleDegrees();
            setTurretAngle(holdAngleDegrees);
        })
        .withName("Manual Align");
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
     * it physically points, then prints results and resets.
     */
    public Command turretDiagnosticCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier cameraHasTarget) {

        return Commands.runOnce(() -> {
            diagStep++;

            switch (diagStep) {
                case 1: // Jog turret to 0°
                    diagLog.setLength(0);
                    setTurretAngle(0);
                    diagLog.append("Step 1: Turret jogged to 0 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagInstruction = "1/3: Turret -> 0 deg. Note where shooter points. Press again.";
                    break;

                case 2: // Jog turret to 130°
                    diagLog.append(String.format("  Encoder settled: %.1f deg\n\n", getTurretAngleDegrees()));
                    setTurretAngle(130);
                    diagLog.append("Step 2: Turret jogged to 130 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagLog.append(String.format("  Camera Has Target: %s\n", cameraHasTarget.getAsBoolean()));
                    diagLog.append(String.format("  Camera Yaw:        %.1f deg\n", cameraAngle.getAsDouble()));
                    diagInstruction = "2/3: Turret -> 130 deg. Note where shooter points. Press again.";
                    break;

                case 3: // Jog turret to 260°
                    diagLog.append(String.format("  Encoder settled: %.1f deg\n\n", getTurretAngleDegrees()));
                    setTurretAngle(260);
                    diagLog.append("Step 3: Turret jogged to 260 deg\n");
                    diagLog.append(String.format("  Encoder reads: %.1f deg\n", getTurretAngleDegrees()));
                    diagLog.append(String.format("  Camera Has Target: %s\n", cameraHasTarget.getAsBoolean()));
                    diagLog.append(String.format("  Camera Yaw:        %.1f deg\n", cameraAngle.getAsDouble()));

                    // Print full results
                    StringBuilder out = new StringBuilder();
                    out.append("\n============ TURRET DIAGNOSTIC FULL RESULTS ============\n");
                    out.append(diagLog);
                    out.append("========================================================\n");
                    System.out.println(out.toString());

                    diagStep = 0;
                    diagLog.setLength(0);
                    diagInstruction = "Done! Paste console output. Press to restart.";
                    return;

                default:
                    diagStep = 0;
                    diagInstruction = "Press 'Turret Diag' to start";
                    return;
            }
        }, this).withName("Turret Diagnostic");
    }
}
