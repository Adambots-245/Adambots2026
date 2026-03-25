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
import com.adambots.lib.sensors.BaseAbsoluteEncoder;
import com.adambots.lib.sensors.DummyAbsoluteEncoder;
import com.adambots.lib.sensors.Potentiometer;
import com.adambots.lib.sensors.ThroughBoreEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

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
    public static final boolean LEDS_ENABLED = true;
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
    private static final int kIntakeArmEncoderPort = 0;  // DIO - throughbore absolute encoder

    // Hardware devices
    // TalonFXMotor(canId, isOnCANivore, isKraken) — supply current is configured in subsystem
    public static final BaseMotor kIntakeMotor = INTAKE_ENABLED
        ? new TalonFXMotor(kIntakeMotorPort, true, true) : new DummyMotor();
    public static final BaseMotor kIntakeMotorArm = INTAKE_ENABLED
        ? new MinionMotor(kIntakeMotorArmPort, true) : new DummyMotor();
    public static final BaseAbsoluteEncoder kIntakeArmEncoder = INTAKE_ENABLED
        ? new ThroughBoreEncoder(kIntakeArmEncoderPort) : new DummyAbsoluteEncoder();

    // ==================== SHOOTER ====================
    // Port assignments - on CANivore
    public static final int kShooterMotor2Port = 25;    // Kraken X60 (leader)
    public static final int kShooterMotor1Port = 24;   // Kraken X60 (follower)

    // Hardware devices
    public static final BaseMotor shooterMotor2 = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterMotor2Port, true, true) : new DummyMotor();
    public static final BaseMotor shooterMotor1 = SHOOTER_ENABLED
        ? new TalonFXMotor(kShooterMotor1Port, true, true) : new DummyMotor();

    // ==================== TURRET ====================
    // Port assignments - on CANivore
    private static final int kTurretPort = 35;        // Minion (WCP GreyT Turret)
    private static final int kTurretPotPort = 0;      // Analog input - 10-turn potentiometer
    private static final double kTurretPotFullRange = 3600.0; // 10-turn pot: 0-3600°

    public static final BaseMotor turretMotor = TURRET_ENABLED
        ? new MinionMotor(kTurretPort, true) : new DummyMotor();
    public static final BaseAbsoluteEncoder kTurretPotentiometer = TURRET_ENABLED
        ? new Potentiometer(kTurretPotPort, kTurretPotFullRange) : new DummyAbsoluteEncoder();

    // ==================== HOPPER (includes uptake motor) ====================
    // Port assignments - on CANivore
    private static final int kHopperPort = 26;
    private static final int kUptakePort = 34;        // Kraken X44

    public static final BaseMotor hopperMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kHopperPort, true, true) : new DummyMotor();
    public static final BaseMotor uptakeMotor = HOPPER_ENABLED
        ? new TalonFXMotor(kUptakePort, true, true) : new DummyMotor();

    // ==================== CLIMBER ====================
    // Port assignments - On CANivore
    private static final int kClimberElevatorMotorPort = 19;
    private static final int kClimberRatchetSolenoidChannel = 0; // Electrical solenoid channel
    private static final int kClimberRaisedLimitPort = 4;   // DIO - limit switch at top
    private static final int kClimberLoweredLimitPort = 5;  // DIO - limit switch at bottom

    // Hardware devices
    public static final BaseMotor kClimberElevatorMotor = CLIMBER_ENABLED
        ? new TalonFXMotor(kClimberElevatorMotorPort, true, true) : new DummyMotor();
    public static final BaseSolenoid kClimberRatchetSolenoid = CLIMBER_ENABLED
        ? new ElectricalSolenoid(kClimberRatchetSolenoidChannel) : new DummySolenoid();
    public static final DigitalInput kClimberRaisedLimit = CLIMBER_ENABLED
        ? new DigitalInput(kClimberRaisedLimitPort) : null;
    public static final DigitalInput kClimberLoweredLimit = CLIMBER_ENABLED
        ? new DigitalInput(kClimberLoweredLimitPort) : null;

    // ==================== LED (CANdle) ====================
    public static final int kCANdlePort = 31;  // CAN ID for CANdle LED controller
}
