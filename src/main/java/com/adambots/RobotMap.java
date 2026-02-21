// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseProximitySensor;
import com.adambots.lib.sensors.LimitSwitch;
import com.adambots.lib.sensors.PhotoEye;

/**
 * RobotMap contains all hardware port mappings and device instantiation for the robot.
 *
 * <p>This class provides a centralized location for all CAN IDs, DIO ports,
 * PWM channels, and creates the actual hardware device objects.
 *
 * <p>Convention: CAN IDs are grouped by subsystem, starting from 13 (after swerve).
 * DIO ports start from 0. PWM channels start from 0.
 *
 * <p>Hardware devices are created here and passed to subsystems via RobotContainer (IoC pattern).
 */
public class RobotMap {

    // ==================== SUBSYSTEM ENABLE FLAGS ====================
    // Set to false for subsystems not yet physically built.
    // When disabled, no CAN/DIO devices are created → no CAN errors.
    public static final boolean INTAKE_ENABLED = false;
    public static final boolean HOPPER_ENABLED = false;
    public static final boolean SHOOTER_ENABLED = false;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean LEDS_ENABLED = true;

    // ==================== CONTROLLER PORTS ====================
    /** Driver controller port - Logitech Extreme 3D Pro joystick */
    public static final int kDriverJoystickPort = 0;
    /** Operator controller port - Xbox controller */
    public static final int kOperatorXboxPort = 1;

    // Swerve drive CAN IDs and IMU are configured via YAGSL JSON files in deploy/swerve/

    // ==================== INTAKE ====================
    // Port assignments
    private static final int kIntakeMotorPort = 13;
    private static final int kIntakeSensorPort = 0;  // DIO

    // Hardware devices
    // TalonFXMotor(canId, inverted, gearRatio, brakeMode)
    public static final BaseMotor kIntakeMotor = INTAKE_ENABLED
        ? new TalonFXMotor(kIntakeMotorPort, false, 1.0, false) : null;
    // PhotoEye(port, inverted)
    public static final BaseProximitySensor kIntakeSensor = INTAKE_ENABLED
        ? new PhotoEye(kIntakeSensorPort, false) : null;

    // ==================== HOPPER ====================
    // Port assignments
    private static final int kHopperCarouselMotorPort = 15;
    private static final int kHopperUptakeMotorPort = 16;
    private static final int kHopperSensorPort = 1;  // DIO

    // Hardware devices
    public static final BaseMotor kHopperCarouselMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kHopperCarouselMotorPort, false, 1.0, false) : null;
    public static final BaseMotor kHopperUptakeMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kHopperUptakeMotorPort, false, 1.0, false) : null;
    public static final BaseProximitySensor kHopperSensor = HOPPER_ENABLED
        ? new PhotoEye(kHopperSensorPort, false) : null;

    // ==================== SHOOTER ====================
    // Port assignments
    private static final int kShooterLeftMotorPort = 17;   // Left flywheel motor (leader)
    private static final int kShooterRightMotorPort = 18;  // Right flywheel motor (follower)
    private static final int kShooterTurretMotorPort = 19; // Turret rotation motor

    // Hardware devices
    // TalonFXMotor(canId, inverted, gearRatio, brakeMode)
    public static final BaseMotor kShooterLeftMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterLeftMotorPort, false, 1.0, false) : null;
    public static final BaseMotor kShooterRightMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterRightMotorPort, true, 1.0, false) : null;  // Inverted to spin opposite
    public static final BaseMotor kShooterTurretMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterTurretMotorPort, false, 1.0, true) : null;  // Brake mode for turret

    // TODO: Add turret encoder (REV Through Bore via DIO)
    // import com.adambots.lib.sensors.ThroughBoreEncoder;
    // public static final ThroughBoreEncoder kTurretEncoder = new ThroughBoreEncoder(ShooterConstants.kTurretEncoderPort);

    // ==================== CLIMBER ====================
    // Port assignments
    private static final int kClimberLeftMotorPort = 20;
    private static final int kClimberRightMotorPort = 21;
    private static final int kClimberLeftLimitPort = 3;   // DIO
    private static final int kClimberRightLimitPort = 4;  // DIO

    // Hardware devices
    public static final BaseMotor kClimberLeftMotor = CLIMBER_ENABLED
        ? new TalonFXMotor(kClimberLeftMotorPort, false, 1.0, true) : null;
    public static final BaseMotor kClimberRightMotor = CLIMBER_ENABLED
        ? new TalonFXMotor(kClimberRightMotorPort, false, 1.0, true) : null;
    // LimitSwitch(port, inverted)
    public static final LimitSwitch kClimberLeftLimit = CLIMBER_ENABLED
        ? new LimitSwitch(kClimberLeftLimitPort, false) : null;
    public static final LimitSwitch kClimberRightLimit = CLIMBER_ENABLED
        ? new LimitSwitch(kClimberRightLimitPort, false) : null;

    // ==================== LED (CANdle) ====================
    public static final int kCANdlePort = 0;  // CAN ID for CANdle LED controller

    // ==================== VISION ====================
    // Camera names are configured in VisionConstants, not here
    // PhotonVision cameras are accessed by name, not port

    // ==================== TEST TURRET (PID TUNING) ====================
    // This is a test subsystem for verifying PIDAutoTuner functionality.
    // Uses a high CAN ID to avoid conflicts with other subsystems.
    private static final int kTestTurretMotorPort = 30;

    // TalonFXMotor(canId, inverted, gearRatio, brakeMode)
    public static final BaseMotor kTestTurretMotor = new TalonFXMotor(
        kTestTurretMotorPort, false, 1.0, true  // Brake mode for position control
    );
}
