// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;
import com.adambots.lib.actuators.DummyMotor;
import com.adambots.lib.actuators.DummySolenoid;
import com.adambots.lib.actuators.ElectricalSolenoid;
import com.adambots.lib.actuators.MinionMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;
import com.adambots.lib.sensors.DummyDistanceSensor;

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
    public static final boolean CLIMBER_ENABLED = true;
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
    // Port assignments - on CANivore
    private static final int kIntakeMotorPort = 33;
    private static final int kIntakeMotorArmPort = 32;

    // Hardware devices
    // TalonFXMotor(canId, isOnCANivore, supplyCurrentLimit, isKraken)
    public static final BaseMotor kIntakeMotor = INTAKE_ENABLED
        ? new TalonFXMotor(kIntakeMotorPort, true, 60.0, true) : new DummyMotor();
    public static final BaseMotor kIntakeMotorArm = INTAKE_ENABLED
        ? new MinionMotor(kIntakeMotorArmPort, true) : new DummyMotor();

    // ==================== SHOOTER ====================
    // Port assignments - on CANivore
    public static final int kShooterLeftPort = 25;    // Kraken X60 (leader)
    public static final int kShooterRightPort = 24;   // Kraken X60 (follower)

    // Hardware devices
    public static final BaseMotor shooterLeftMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterLeftPort, true, 60.0, true) : new DummyMotor();
    public static final BaseMotor shooterRightMotor = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterRightPort, true, 60.0, true) : new DummyMotor();

    // ==================== TURRET ====================
    // Port assignments - on CANivore
    private static final int kTurretPort = 35;        // Minion (WCP GreyT Turret)

    public static final BaseMotor turretMotor = TURRET_ENABLED
        ? new MinionMotor(kTurretPort, true) : new DummyMotor();

    // ==================== HOPPER (includes uptake motor) ====================
    // Port assignments - on CANivore
    private static final int kHopperPort = 26;
    private static final int kUptakePort = 34;        // Kraken X44
    private static final int kHopperSensorPort = 27;  // CANRange sensor

    public static final BaseMotor hopperMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kHopperPort, true, 20.0, true) : new DummyMotor();
    public static final BaseMotor uptakeMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kUptakePort, true, 40.0, true) : new DummyMotor();
    // TODO: Re-enable when CANRange is wired in
    // public static final BaseDistanceSensor hopperSensor = HOPPER_ENABLED
    //     ? new CANRangeSensor(kHopperSensorPort, true) : new DummyDistanceSensor();
    public static final BaseDistanceSensor hopperSensor = new DummyDistanceSensor();

    // ==================== CLIMBER ====================
    // Port assignments - On CANivore
    private static final int kClimberElevatorMotorPort = 16;
    private static final int kClimberRatchetSolenoidChannel = 4; // Electrical solenoid channel

    // Hardware devices
    public static final BaseMotor kClimberElevatorMotor = CLIMBER_ENABLED
        ? new TalonFXMotor(kClimberElevatorMotorPort, true, 60.0, true) : new DummyMotor();
    public static final BaseSolenoid kClimberRatchetSolenoid = CLIMBER_ENABLED
        ? new ElectricalSolenoid(kClimberRatchetSolenoidChannel) : new DummySolenoid();

    // ==================== LED (CANdle) ====================
    public static final int kCANdlePort = 31;  // CAN ID for CANdle LED controller
}
