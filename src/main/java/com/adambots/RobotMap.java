package com.adambots;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.MinionMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;
import com.adambots.lib.sensors.CANRangeSensor;

/**
 * RobotMap defines all hardware port assignments and motor instances for TestBoard.
 *
 * This is a subsystem testing platform - add your motor CAN IDs here.
 * Motors are created here and passed to subsystems via constructor injection.
 *
 * CAN ID Assignment Guidelines:
 *   0-9:   Reserved for drivetrain (if ever added)
 *   10-19: Intake/indexer mechanisms
 *   20-29: Shooter/launcher mechanisms
 *   30-39: Arm/elevator mechanisms
 *   40-49: Climber mechanisms
 *   50+:   Miscellaneous
 */
public class RobotMap {

    // ==================== CAN IDs ====================
    // Shooter Prototype
    public static final int kShooterLeftPort = 21;   // Kraken X60
    public static final int kShooterRightPort = 22;  // Kraken X60 (follower)
    public static final int kTurretPort = 51;        // Minion (WCP GreyT Turret)
    public static final int kUptakePort = 20;        // Kraken X44
    public static final int kHopperPort = 26;        // Kraken X60
    public static final int kCANRangeHopperPort = 35; 
    // ==================== Motor Instances ====================
    // Using BaseMotor allows easy swap between motor types (TalonFX, NEO, etc.)
    // TalonFXMotor(portNum, isOnCANivore, supplyCurrentLimit, isKraken)

    // Shooter motors (Kraken X60)
    public static final BaseMotor shooterLeftMotor = new TalonFXMotor(kShooterLeftPort, false, 60.0, true);
    public static final BaseMotor shooterRightMotor = new TalonFXMotor(kShooterRightPort, false, 60.0, true);

    // Turret motor (Minion)
    public static final BaseMotor turretMotor = new MinionMotor(kTurretPort);

    // Uptake motor (Kraken X44)
    public static final BaseMotor uptakeMotor = new TalonFXMotor(kUptakePort, false, 40.0, true);

    // Hopper motor (Kraken X60)
    public static final BaseMotor hopperMotor = new TalonFXMotor(kHopperPort, false, 60.0, true);

    // CANRange for Hopper
    public static final BaseDistanceSensor hopperPieceSensor = new CANRangeSensor(kCANRangeHopperPort, false);

    // ==================== Add Your Motors Here ====================
    // Example:
    // public static final int kIntakePort = 10;
    // public static final BaseMotor intakeMotor = new TalonFXMotor(kIntakePort);
    //
    // public static final int kArmLeftPort = 30;
    // public static final int kArmRightPort = 31;
    // public static final BaseMotor armLeftMotor = new TalonFXMotor(kArmLeftPort);
    // public static final BaseMotor armRightMotor = new TalonFXMotor(kArmRightPort);
}