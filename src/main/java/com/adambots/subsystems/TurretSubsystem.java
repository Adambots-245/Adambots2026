package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import static edu.wpi.first.units.Units.Degrees;

import com.adambots.Constants;
import com.adambots.Constants.SimulationConstants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.sensors.BaseAbsoluteEncoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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

    // Scan direction for SWEEP mode: +1 = toward max, -1 = toward zero
    private int scanDirection = 1;

    // Simulation
    private SingleJointedArmSim turretSim;

    public TurretSubsystem(BaseMotor turretMotor, BaseAbsoluteEncoder turretPot) {
        this.turretMotor = turretMotor;
        this.turretPot = turretPot;
        configureMotors();

        // Seed motor encoder from potentiometer absolute position
        double absoluteDeg = getPotAngleDegrees();
        double rotations = (absoluteDeg / 360.0) * TurretConstants.kTurretGearRatio;
        turretMotor.setPosition(rotations);

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
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

    // ==================== Simulation ====================

    private void setupSimulation() {
        turretSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            TurretConstants.kTurretGearRatio,
            SimulationConstants.kTurretMOI,
            0.3, // arm length (visual only)
            Math.toRadians(-TurretConstants.kTurretMaxDegrees),
            Math.toRadians(TurretConstants.kTurretMaxDegrees),
            false, // no gravity for horizontal turret
            0.0);
        turretMotor.setPosition(0);
    }

    @Override
    public void simulationPeriodic() {
        if (turretSim == null) return;

        double voltage = turretMotor.getSimMotorVoltage();
        turretSim.setInputVoltage(voltage);
        turretSim.update(0.02);

        double angleRot = Math.toDegrees(turretSim.getAngleRads()) / 360.0 * TurretConstants.kTurretGearRatio;
        double velRotPerSec = Math.toDegrees(turretSim.getVelocityRadPerSec()) / 360.0 * TurretConstants.kTurretGearRatio;
        turretMotor.setSimPosition(angleRot);
        turretMotor.setSimVelocity(velRotPerSec);

        Logger.recordOutput("Components", new Pose3d[] {
            new Pose3d(
                new Translation3d(0, 0, 0.15),
                new Rotation3d(0, 0, Math.toRadians(getTurretAngleDegrees())))
        });
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
        return autoTrackCommand(cameraAngle, hubVisible, hubFresh, inShootingZone, () -> 0.0);
    }

    public Command autoTrackCommand(
            DoubleSupplier cameraAngle,
            BooleanSupplier hubVisible,
            BooleanSupplier hubFresh,
            BooleanSupplier inShootingZone,
            DoubleSupplier robotAngularVelDegPerSec) {

        return Commands.runOnce(() -> {
            holdAngleDegrees = getTurretAngleDegrees();
            wasAutoTracking = false;
            scanDirection = 1;
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

            // Angular velocity lead: compensate for robot rotation so turret holds aim.
            // Robot rotating CW (positive omega) → turret must lead CCW (negative in turret frame).
            double angVelLead = -robotAngularVelDegPerSec.getAsDouble()
                * TurretTrackingConstants.kAngularVelLeadTime;

            double currentAngle = getTurretAngleDegrees();

            if (hubVisible.getAsBoolean()) {
                // CAMERA MODE — hub visible (camera detection or pose-derived)
                trackingMode = TrackingMode.CAMERA;
                // Remember which side the hub is for faster reacquisition
                scanDirection = (cameraAngle.getAsDouble() >= 0) ? 1 : -1;
                // Compute target from current angle + turret-relative offset
                double rawTarget = currentAngle + cameraAngle.getAsDouble();
                // If target is past mechanical limits, hub is unreachable from this position.
                // Hold at the limit rather than tracking stale corrections against the stop.
                if (rawTarget < 0 || rawTarget > TurretConstants.kTurretMaxDegrees) {
                    setTurretAngle(MathUtil.clamp(rawTarget, 0, TurretConstants.kTurretMaxDegrees));
                } else if (hubFresh.getAsBoolean()) {
                    // Fresh camera detection — apply correction at full tracking gain
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
                    setTurretAngle(smoothed + angVelLead);
                } else {
                    // No fresh camera frame — use pose-derived angle (via getHubAngle)
                    // at half gain to slew toward hub without overshooting on stale data
                    double smoothed = lastSetpointDegrees
                        + (rawTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain * 0.5;
                    setTurretAngle(smoothed + angVelLead);
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
                        + (rawTarget - lastSetpointDegrees) * TurretTrackingConstants.kCameraTrackingGain;
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
