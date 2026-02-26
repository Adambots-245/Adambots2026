// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MinionMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;
import com.adambots.lib.sensors.BaseProximitySensor;
import com.adambots.lib.sensors.CANRangeSensor;
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
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean HOPPER_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean TURRET_ENABLED = true;
    // Uptake motor is owned by HopperSubsystem — no separate enable flag needed
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean LEDS_ENABLED = false;
    public static final boolean BACK_CAMERAS_ENABLED = true;
    public static final boolean SHOOTER_CAMERA_ENABLED = true;

    // ==================== CONTROLLER PORTS ====================
    /** Driver controller port - Logitech Extreme 3D Pro joystick */
    public static final int kDriverJoystickPort = 0;
    /** Operator controller port - Xbox controller */
    public static final int kOperatorXboxPort = 1;

    // Swerve drive CAN IDs and IMU are configured via YAGSL JSON files in deploy/swerve/

    // ==================== INTAKE ====================
    // Port assignments
    private static final int kIntakeMotorPort = 33;
    private static final int kIntakeMotorArmPort = 32;

    // Hardware devices
    // TalonFXMotor(canId, isOnCANivore, supplyCurrentLimit, isKraken)
    public static final BaseMotor kIntakeMotor = INTAKE_ENABLED
        ? new TalonFXMotor(kIntakeMotorPort, false, 1.0, false) : null;
    public static final BaseMotor kIntakeMotorArm = INTAKE_ENABLED
        ? new TalonFXMotor(kIntakeMotorArmPort, false, 1.0, false) : null;

    // ==================== SHOOTER ====================
    // Port assignments
    public static final int kShooterLeftPort = 24;    // Kraken X60 (leader)
    public static final int kShooterRightPort = 25;   // Kraken X60 (follower)

    // Hardware devices
    public static final BaseMotor shooterLeftMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterLeftPort, false, 60.0, true) : null;
    public static final BaseMotor shooterRightMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterRightPort, false, 60.0, true) : null;

    // ==================== TURRET ====================
    private static final int kTurretPort = 35;        // Minion (WCP GreyT Turret)

    public static final BaseMotor turretMotor = TURRET_ENABLED
        ? new MinionMotor(kTurretPort) : null;

    // ==================== HOPPER (includes uptake motor) ====================
    private static final int kHopperPort = 26;
    private static final int kUptakePort = 34;        // Kraken X44
    private static final int kHopperSensorPort = 27;  // CANRange sensor

    public static final BaseMotor hopperMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kHopperPort, false, 60.0, true) : null;
    public static final BaseMotor uptakeMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kUptakePort, false, 40.0, true) : null;
    public static final BaseDistanceSensor hopperSensor = HOPPER_ENABLED
        ? new CANRangeSensor(kHopperSensorPort, false) : null;

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
}
