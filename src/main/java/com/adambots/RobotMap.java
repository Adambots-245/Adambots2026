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

    // ==================== CONTROLLER PORTS ====================
    /** Driver controller port - Logitech Extreme 3D Pro joystick */
    public static final int kDriverJoystickPort = 0;
    /** Operator controller port - Xbox controller */
    public static final int kOperatorXboxPort = 1;

    // ==================== SWERVE DRIVE CAN IDs ====================
    // Front Left Module
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftTurnMotorPort = 2;
    public static final int kFrontLeftEncoderPort = 9;

    // Front Right Module
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kFrontRightTurnMotorPort = 4;
    public static final int kFrontRightEncoderPort = 10;

    // Back Left Module
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackLeftTurnMotorPort = 6;
    public static final int kBackLeftEncoderPort = 11;

    // Back Right Module
    public static final int kBackRightDriveMotorPort = 7;
    public static final int kBackRightTurnMotorPort = 8;
    public static final int kBackRightEncoderPort = 12;

    // ==================== IMU ====================
    public static final int kPigeonPort = 0;

    // ==================== INTAKE ====================
    // Port assignments
    private static final int kIntakeMotorPort = 13;
    private static final int kIntakeSensorPort = 0;  // DIO

    // Hardware devices
    // TalonFXMotor(canId, inverted, gearRatio, brakeMode)
    public static final BaseMotor kIntakeMotor = new TalonFXMotor(kIntakeMotorPort, false, 1.0, false);
    // PhotoEye(port, inverted)
    public static final BaseProximitySensor kIntakeSensor = new PhotoEye(kIntakeSensorPort, false);

    // ==================== HOPPER ====================
    // Port assignments
    private static final int kHopperCarouselMotorPort = 15;
    private static final int kHopperUptakeMotorPort = 16;
    private static final int kHopperSensorPort = 1;  // DIO

    // Hardware devices
    public static final BaseMotor kHopperCarouselMotor = new TalonFXMotor(kHopperCarouselMotorPort, false, 1.0, false);
    public static final BaseMotor kHopperUptakeMotor = new TalonFXMotor(kHopperUptakeMotorPort, false, 1.0, false);
    public static final BaseProximitySensor kHopperSensor = new PhotoEye(kHopperSensorPort, false);

    // ==================== SHOOTER ====================
    // Port assignments
    private static final int kShooterLeftMotorPort = 17;   // Left flywheel motor (leader)
    private static final int kShooterRightMotorPort = 18;  // Right flywheel motor (follower)
    private static final int kShooterTurretMotorPort = 19; // Turret rotation motor

    // Hardware devices
    // TalonFXMotor(canId, inverted, gearRatio, brakeMode)
    public static final BaseMotor kShooterLeftMotor = new TalonFXMotor(kShooterLeftMotorPort, false, 1.0, false);
    public static final BaseMotor kShooterRightMotor = new TalonFXMotor(kShooterRightMotorPort, true, 1.0, false);  // Inverted to spin opposite
    public static final BaseMotor kShooterTurretMotor = new TalonFXMotor(kShooterTurretMotorPort, false, 1.0, true);  // Brake mode for turret

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
    public static final BaseMotor kClimberLeftMotor = new TalonFXMotor(kClimberLeftMotorPort, false, 1.0, true);
    public static final BaseMotor kClimberRightMotor = new TalonFXMotor(kClimberRightMotorPort, false, 1.0, true);
    // LimitSwitch(port, inverted)
    public static final LimitSwitch kClimberLeftLimit = new LimitSwitch(kClimberLeftLimitPort, false);
    public static final LimitSwitch kClimberRightLimit = new LimitSwitch(kClimberRightLimitPort, false);

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
