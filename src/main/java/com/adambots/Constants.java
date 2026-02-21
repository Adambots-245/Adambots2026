package com.adambots;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * Constants for TestBoard subsystem testing platform.
 *
 * Add constants for your prototype mechanisms here.
 * Use WPILib Units library for type-safe measurements.
 */
public final class Constants {

    /**
     * Shooter test constants for PID velocity-controlled flywheel + position-controlled turret.
     * Reference values from Adambots2026 simulation work.
     */
    public static final class ShooterTestConstants {
        // ==================== Flywheel Motor Specs ====================
        // Kraken X60 free speed: ~6000 RPM = 100 RPS at 12V
        // Update these if you swap motor type
        public static final double kMotorFreeSpeedRPS = 100.0;
        public static final double kNominalVoltage = 12.0;

        // Set to -1.0 to reverse flywheel direction (workaround for setInverted issue)
        public static final double kFlywheelDirection = -1.0;

        // ==================== Flywheel PID Defaults ====================
        public static final double kFlywheelP = 0.1;
        public static final double kFlywheelI = 0;
        public static final double kFlywheelD = 0;

        // Feedforward: kF = nominalVoltage / motorFreeSpeedRPS
        // = 12.0 / 100.0 = 0.12 V/RPS
        // Verified against ReCalc flywheel calculator (0.37 V*s/m surface velocity
        // * 2*pi*0.0508m wheel radius = 0.118 V/RPS -- matches within rounding)
        // Recalculate if you change motor type, gear ratio, or voltage
        public static final double kFlywheelFF = kNominalVoltage / kMotorFreeSpeedRPS;

        // ==================== Turret PID Defaults (from Adambots2026 simulation) ====================
        public static final double kTurretP = 0.75;
        public static final double kTurretI = 0;
        public static final double kTurretD = 0;
        public static final double kTurretFF = 0.2;

        // ==================== Turret Mechanical ====================
        // WCP GreyT Turret gear ratio (motor rotations per turret rotation)
        // Default 100:1 -- UPDATE THIS once you confirm the actual ratio
        // from your gearbox configuration (ring gear teeth / pinion teeth * gearbox stages)
        public static final double kTurretGearRatio = 200.0/18.0; //was 100.0, 20 was mechs estimate 
        //200 big wheel 18 small wheel 
        public static final double kTurretMinDegrees = 0.0;
        public static final double kTurretMaxDegrees = 120.0;

        // Soft limit rotations derived from degrees/360 * gearRatio
        public static final double kTurretForwardLimit = (kTurretMaxDegrees / 360.0) * kTurretGearRatio;
        public static final double kTurretReverseLimit = (kTurretMinDegrees / 360.0) * kTurretGearRatio;

        // ==================== Physics Constants ====================
        public static final double kHoodAngleDegrees = 60.0;
        public static final double kFlywheelRadiusMeters = 0.0508;
        public static final double kExitVelocityMultiplier = 0.85;
        public static final double kHubHeightMeters = 1.83;
        public static final double kMaxFreeSpeedRPS = kMotorFreeSpeedRPS;
        public static final double kFuelMassKg = 0.227;
        public static final double kFuelDiameterMeters = 0.15;
        public static final double kGravity = 9.81;

        // EXIT HEIGHT - MUST BE MEASURED ON ACTUAL ROBOT
        // How to measure: With the shooter mounted at the real robot height,
        // place a ball in the shooter at the exit point. Measure from the
        // floor to the center of the ball. Record in meters.
        // Example: 18 inches = 0.4572 meters
        public static final double kExitHeightMeters = 0.48;

        // ==================== Flywheel Tolerance ====================
        public static final double kFlywheelToleranceRPS = 2.0;

        // ==================== Default Interpolation Table ====================
        // Starting guesses for distance (meters) -> RPS, will be tuned
        public static final double[][] kDefaultInterpolationTable = {
            {2.0, 45.0},
            {2.5, 46.5},
            {3.0, 48.0},
            {4.0, 54.0},
            {5.0, 60.0}
        };

        // ==================== Current Limits ====================
        public static final double kFlywheelStallCurrentLimit = 40.0;
        public static final double kFlywheelFreeCurrentLimit = 60.0;
        public static final double kTurretStallCurrentLimit = 20.0;
        public static final double kTurretFreeCurrentLimit = 40.0;
    }

    /**
     * Uptake constants for the ball feed motor.
     */
    public static final class UptakeConstants {
        // Motor speed (duty cycle -1.0 to 1.0)
        public static final double kUptakeSpeed = 0.5;

        // Current limit
        public static final Current kUptakeCurrentLimit = Amps.of(40);
    }


    public static final class HopperConstants{
        public static final double kHopperSpeed = 0.5;
        public static final double kDetectionRange = 2.0; //cm
    }

    // ==================== Add Your Constants Here ====================
    // Example:
    // public static final class IntakeConstants {
    //     public static final double kIntakeSpeed = 0.8;
    //     public static final Current kIntakeCurrentLimit = Amps.of(30);
    // }
    //
    // public static final class ArmConstants {
    //     public static final Angle kMinAngle = Degrees.of(0);
    //     public static final Angle kMaxAngle = Degrees.of(90);
    //     public static final double kP = 1.0;
    //     public static final double kI = 0.0;
    //     public static final double kD = 0.0;
    // }
}