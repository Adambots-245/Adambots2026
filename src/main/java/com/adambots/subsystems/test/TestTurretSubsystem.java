// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems.test;

import com.adambots.Constants.TestTurretConstants;
import com.adambots.lib.actuators.BaseMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Test turret subsystem for verifying PIDAutoTuner functionality.
 *
 * <p>This subsystem provides a simple turret mechanism with:
 * <ul>
 *   <li>Position-based control (angle in degrees)</li>
 *   <li>Simulated physics for realistic response in simulation mode</li>
 *   <li>Methods compatible with PIDAutoTuner API</li>
 *   <li>Mechanism2d visualization for real-time visual feedback</li>
 * </ul>
 *
 * <p>PIDAutoTuner-compatible methods:
 * <ul>
 *   <li>{@link #getAngle()} - DoubleSupplier for measurement</li>
 *   <li>{@link #setPercentOutput(double)} - DoubleConsumer for control</li>
 * </ul>
 */
public class TestTurretSubsystem extends SubsystemBase {

    // ==================== HARDWARE ====================
    private final BaseMotor turretMotor;

    // ==================== SIMULATION STATE ====================
    // For simulation - tracks simulated angle and physics
    private double simulatedAngle = 0;
    private double simulatedVelocity = 0;

    // ==================== PID CONTROL ====================
    private final PIDController positionPID;
    private double targetAngle = 0;
    private boolean pidEnabled = false;

    // ==================== MECHANISM2D VISUALIZATION ====================
    // Creates a visual representation of the turret that can be viewed in
    // SmartDashboard, Shuffleboard, or AdvantageScope
    private final Mechanism2d turretMech2d;
    private final MechanismRoot2d turretRoot;
    private final MechanismLigament2d turretArm;
    private final MechanismLigament2d targetIndicator;

    // Track oscillation state for color feedback
    private double lastAngle = 0;
    private int directionChangeCount = 0;
    private boolean wasIncreasing = false;
    private static final int OSCILLATION_THRESHOLD = 4;  // Direction changes to consider "oscillating"

    // ==================== CONSTRUCTOR ====================
    /**
     * Creates a new TestTurretSubsystem.
     *
     * @param turretMotor The turret rotation motor controller
     */
    public TestTurretSubsystem(BaseMotor turretMotor) {
        this.turretMotor = turretMotor;

        // Initialize position PID with default values
        positionPID = new PIDController(
            TestTurretConstants.kP,
            TestTurretConstants.kI,
            TestTurretConstants.kD
        );
        positionPID.setTolerance(2.0);  // 2 degree tolerance
        // Note: NOT using enableContinuousInput because the turret has physical limits
        // and cannot wrap around from 180° to -180°

        // ==================== MECHANISM2D SETUP ====================
        // Create 2D canvas (width=2, height=2, centered at 1,1)
        turretMech2d = new Mechanism2d(2, 2);

        // Create pivot point at center
        turretRoot = turretMech2d.getRoot("turret", 1, 1);

        // Create turret arm (0.8m long, starts at 0 degrees)
        turretArm = turretRoot.append(
            new MechanismLigament2d("arm", 0.8, 0, 6, new Color8Bit(Color.kCyan))
        );

        // Create target indicator (lighter, shows where we're aiming)
        targetIndicator = turretRoot.append(
            new MechanismLigament2d("target", 0.85, 0, 2, new Color8Bit(Color.kGray))
        );

        // Publish to SmartDashboard for visualization
        SmartDashboard.putData("PID Tuning/Turret Mechanism", turretMech2d);
    }

    // ==================== PIDAUTOTUNER-COMPATIBLE METHODS ====================

    /**
     * Gets the current turret angle in degrees.
     * This is the DoubleSupplier measurement method for PIDAutoTuner.
     *
     * @return The turret angle (0 = forward, positive = counterclockwise)
     */
    public double getAngle() {
        if (RobotBase.isSimulation()) {
            return simulatedAngle;
        }
        // Real robot: get position from motor encoder
        return turretMotor.getPosition() * 360.0 / TestTurretConstants.kGearRatio;
    }

    /**
     * Sets the turret motor to a percent output.
     * This is the DoubleConsumer output method for PIDAutoTuner.
     *
     * @param output Motor output from -1 to 1
     */
    public void setPercentOutput(double output) {
        // Clamp output to safe range
        output = Math.max(-1.0, Math.min(1.0, output));
        turretMotor.set(output);

        // In simulation, update physics
        if (RobotBase.isSimulation()) {
            updateSimulatedPhysics(output);
        }
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * Returns a command that aims the turret to a specific angle.
     * Uses internal PID control.
     *
     * @param angleDegrees Target angle in degrees
     */
    public Command aimToAngleCommand(double angleDegrees) {
        return run(() -> {
            targetAngle = angleDegrees;
            pidEnabled = true;
        })
        .finallyDo(() -> {
            pidEnabled = false;
            turretMotor.set(0);
        })
        .withName("AimTurret(" + angleDegrees + ")");
    }

    /**
     * Returns a command that stops the turret.
     */
    public Command stopCommand() {
        return runOnce(() -> {
            pidEnabled = false;
            turretMotor.set(0);
        }).withName("StopTurret");
    }

    /**
     * Returns a command that manually controls the turret with percent output.
     *
     * @param outputSupplier Supplies the percent output (-1 to 1)
     */
    public Command manualControlCommand(java.util.function.DoubleSupplier outputSupplier) {
        return run(() -> {
            pidEnabled = false;
            setPercentOutput(outputSupplier.getAsDouble());
        })
        .finallyDo(() -> turretMotor.set(0))
        .withName("ManualTurret");
    }

    // ==================== TRIGGERS ====================

    /**
     * Returns true when the turret is at the target angle.
     * Checks directly against targetAngle field rather than PID setpoint
     * to avoid timing issues where PID setpoint hasn't been updated yet.
     */
    public Trigger isAtTargetTrigger() {
        return new Trigger(() -> pidEnabled && Math.abs(getAngle() - targetAngle) < 2.0);
    }

    /**
     * Returns true when the turret is near center (within 5 degrees of 0).
     */
    public Trigger isCenteredTrigger() {
        return new Trigger(() -> Math.abs(getAngle()) < 5.0);
    }

    // ==================== PID GAIN SETTERS ====================

    /**
     * Sets the PID gains for position control.
     * Use this after PIDAutoTuner completes to apply the tuned gains.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPIDGains(double kP, double kI, double kD) {
        positionPID.setPID(kP, kI, kD);
        SmartDashboard.putNumber("PID Tuning/TestTurret/kP", kP);
        SmartDashboard.putNumber("PID Tuning/TestTurret/kI", kI);
        SmartDashboard.putNumber("PID Tuning/TestTurret/kD", kD);
    }

    /**
     * Gets the internal PID controller for direct access.
     * Useful for applying TuningResult gains.
     *
     * @return The position PID controller
     */
    public PIDController getPIDController() {
        return positionPID;
    }

    /**
     * Gets the motor for direct access.
     * Useful for applying TuningResult gains to motor slot.
     *
     * @return The turret motor
     */
    public BaseMotor getMotor() {
        return turretMotor;
    }

    // ==================== PERIODIC ====================
    @Override
    public void periodic() {
        // Run PID control if enabled
        if (pidEnabled) {
            double currentAngle = getAngle();
            double output = positionPID.calculate(currentAngle, targetAngle);
            setPercentOutput(output);
        }

        // Update mechanism visualization
        updateMechanism2d();

        // Publish telemetry
        SmartDashboard.putNumber("PID Tuning/TestTurret/Angle", getAngle());
        SmartDashboard.putNumber("PID Tuning/TestTurret/Target", targetAngle);
        SmartDashboard.putBoolean("PID Tuning/TestTurret/AtTarget", positionPID.atSetpoint());
        SmartDashboard.putBoolean("PID Tuning/TestTurret/PIDEnabled", pidEnabled);
    }

    // ==================== PRIVATE HELPERS ====================

    /**
     * Updates simulated physics based on motor output.
     * Simple second-order system with inertia and friction.
     */
    private void updateSimulatedPhysics(double motorOutput) {
        // Time step (20ms periodic)
        double dt = 0.02;

        // Motor torque produces angular acceleration (simplified)
        // τ = I * α, where motor output produces torque proportional to voltage
        double maxAngularAccel = 1000.0;  // deg/s^2 at full power
        double angularAccel = motorOutput * maxAngularAccel;

        // Apply friction (opposes velocity)
        double frictionDecel = simulatedVelocity * TestTurretConstants.kFrictionCoefficient * 100;
        angularAccel -= frictionDecel;

        // Update velocity and position
        simulatedVelocity += angularAccel * dt;
        simulatedAngle += simulatedVelocity * dt;

        // Clamp to safe range
        if (simulatedAngle > TestTurretConstants.kMaxAngle) {
            simulatedAngle = TestTurretConstants.kMaxAngle;
            simulatedVelocity = 0;
        } else if (simulatedAngle < TestTurretConstants.kMinAngle) {
            simulatedAngle = TestTurretConstants.kMinAngle;
            simulatedVelocity = 0;
        }
    }

    /**
     * Updates the Mechanism2d visualization.
     */
    private void updateMechanism2d() {
        double currentAngle = getAngle();

        // Update arm angle from current position
        turretArm.setAngle(currentAngle);

        // Update target indicator
        targetIndicator.setAngle(targetAngle);

        // Track direction changes for oscillation detection
        boolean isIncreasing = currentAngle > lastAngle;
        if (isIncreasing != wasIncreasing && Math.abs(currentAngle - lastAngle) > 0.5) {
            directionChangeCount++;
        }
        wasIncreasing = isIncreasing;
        lastAngle = currentAngle;

        // Decay direction change count over time
        if (directionChangeCount > 0 && Math.abs(simulatedVelocity) < 1.0) {
            directionChangeCount = Math.max(0, directionChangeCount - 1);
        }

        // Update color based on state:
        // - Green: at target
        // - Yellow: moving
        // - Red: oscillating (during PID tuning)
        if (directionChangeCount >= OSCILLATION_THRESHOLD) {
            turretArm.setColor(new Color8Bit(Color.kRed));
        } else if (pidEnabled && positionPID.atSetpoint()) {
            turretArm.setColor(new Color8Bit(Color.kGreen));
        } else if (Math.abs(simulatedVelocity) > 5.0 || pidEnabled) {
            turretArm.setColor(new Color8Bit(Color.kYellow));
        } else {
            turretArm.setColor(new Color8Bit(Color.kCyan));
        }
    }

    /**
     * Resets the oscillation detection counter.
     * Call this when starting a new tuning session.
     */
    public void resetOscillationDetection() {
        directionChangeCount = 0;
    }
}
