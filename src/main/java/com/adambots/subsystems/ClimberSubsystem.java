// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.ClimberConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem — single Kraken X60 elevator + solenoid ratchet.
 *
 * <p>The ratchet is spring-return: de-energized = engaged (holds position),
 * energized = released (allows motor movement).
 */
public class ClimberSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor elevatorMotor;
    private final BaseSolenoid ratchetSolenoid;
    private final DigitalInput raisedLimit;   // DIO — true when NOT pressed (normally open)
    private final DigitalInput loweredLimit;  // DIO — true when NOT pressed (normally open)

    // ==================== SECTION: CONSTRUCTOR ====================
    public ClimberSubsystem(BaseMotor elevatorMotor, BaseSolenoid ratchetSolenoid,
                            DigitalInput raisedLimit, DigitalInput loweredLimit) {
        this.elevatorMotor = elevatorMotor;
        this.ratchetSolenoid = ratchetSolenoid;
        this.raisedLimit = raisedLimit;
        this.loweredLimit = loweredLimit;

        configureMotors();
        engageRatchet();
    }

    private void configureMotors() {
        elevatorMotor.configure()
            .brakeMode(true)
            .inverted(true)
            .currentLimits(ClimberConstants.kStatorCurrentLimit,
                           ClimberConstants.kSupplyCurrentLimit, 3000)
            .apply();
    }

    // ==================== SECTION: SOLENOID HELPERS ====================
    private void releaseRatchet() {
        ratchetSolenoid.set(true);
    }

    private void engageRatchet() {
        ratchetSolenoid.set(false);
    }

    public boolean isRatchetEngaged() {
        return !ratchetSolenoid.get();
    }

    // ==================== SECTION: LIMIT SWITCHES ====================

    /** Returns true when the raised (upper) limit switch is pressed. */
    public boolean isAtRaisedLimit() {
        return raisedLimit != null && !raisedLimit.get();  // DIO reads false when closed
    }

    /** Returns true when the lowered (bottom) limit switch is pressed. */
    public boolean isAtLoweredLimit() {
        return loweredLimit != null && !loweredLimit.get();  // DIO reads false when closed
    }

    // ==================== SECTION: COMMAND FACTORIES ====================

    /** Extend elevator upward. Releases ratchet once at start, engages on end. Stops at raised limit. */
    public Command extendCommand() {
        return startRun(
            this::releaseRatchet,
            () -> {
                if (!isAtRaisedLimit()) {
                    elevatorMotor.set(ClimberConstants.kExtendSpeed);
                } else {
                    elevatorMotor.set(0);
                    engageRatchet();
                }
            }
        ).finallyDo(() -> {
            elevatorMotor.set(0);
            engageRatchet();
        }).withName("ExtendClimber");
    }

    /** Retract elevator downward. Releases ratchet once at start, engages on end. Stops at lowered limit. */
    public Command retractCommand() {
        return startRun(
            // this::releaseRatchet,
            () -> {},
            () -> {
                if (!isAtLoweredLimit()) {
                    elevatorMotor.set(-ClimberConstants.kClimbSpeed);
                } else {
                    elevatorMotor.set(0);
                    engageRatchet();
                }
            }
        ).finallyDo(() -> {
            elevatorMotor.set(0);
            engageRatchet();
        }).withName("RetractClimber");
    }

    /** Lower robot from hang at controlled speed. Gravity assists, so use reduced motor output. */
    public Command lowerCommand() {
        return startRun(
            this::releaseRatchet,
            () -> {
                if (!isAtLoweredLimit()) {
                    elevatorMotor.set(ClimberConstants.kLowerSpeed);
                } else {
                    elevatorMotor.set(0);
                    engageRatchet();
                }
            }
        ).finallyDo(() -> {
            elevatorMotor.set(0);
            engageRatchet();
        }).withName("LowerClimber");
    }

    /** Alias for retractCommand — pulls the robot up. */
    public Command climbCommand() {
        return retractCommand().withName("Climb");
    }

    /** Stop motor and engage ratchet to hold position. */
    public Command lockCommand() {
        return runOnce(() -> {
            elevatorMotor.set(0);
            engageRatchet();
        }).withName("LockClimber");
    }

    /** Engage ratchet (hold position). */
    public Command engageRatchetCommand() {
        return runOnce(this::engageRatchet).withName("EngageRatchet");
    }

    /** Release ratchet (allow motor movement). */
    public Command releaseRatchetCommand() {
        return runOnce(this::releaseRatchet).withName("ReleaseRatchet");
    }

    // ==================== SECTION: PERIODIC ====================
    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();

        // Safety: stop motor if at limit in the direction it's moving
        double output = elevatorMotor.getOutputPercent();
        if ((output > 0 && isAtRaisedLimit()) || (output < 0 && isAtLoweredLimit())) {
            elevatorMotor.set(0);
            engageRatchet();
        }

        if (Constants.CURRENT_LOGGING) {
            Logger.recordOutput("Climber/ElevatorCurrent", elevatorMotor.getCurrentDraw().in(Amps));
        }

        if (Constants.TUNING_ENABLED) {
            Logger.recordOutput("Climber/Position", elevatorMotor.getPosition());
            Logger.recordOutput("Climber/RatchetEngaged", isRatchetEngaged());
            Logger.recordOutput("Climber/RaisedLimit", isAtRaisedLimit());
            Logger.recordOutput("Climber/LoweredLimit", isAtLoweredLimit());
            Logger.recordOutput("Timing/ClimberSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
        }
    }
}
