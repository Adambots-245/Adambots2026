// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ClimberConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseSolenoid;

import edu.wpi.first.epilogue.Logged;
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
@Logged
public class ClimberSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor elevatorMotor;
    private final BaseSolenoid ratchetSolenoid;

    // ==================== SECTION: CONSTRUCTOR ====================
    public ClimberSubsystem(BaseMotor elevatorMotor, BaseSolenoid ratchetSolenoid) {
        this.elevatorMotor = elevatorMotor;
        this.ratchetSolenoid = ratchetSolenoid;

        configureMotors();
        engageRatchet();
    }

    private void configureMotors() {
        elevatorMotor.configure()
            .brakeMode(true)
            .currentLimits(ClimberConstants.kStallCurrentLimit,
                           ClimberConstants.kFreeCurrentLimit, 3000)
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

    // ==================== SECTION: COMMAND FACTORIES ====================

    /** Extend elevator upward. Releases ratchet while running, engages on end. */
    public Command extendCommand() {
        return runEnd(
            () -> {
                releaseRatchet();
                elevatorMotor.set(ClimberConstants.kExtendSpeed);
            },
            () -> {
                elevatorMotor.set(0);
                engageRatchet();
            }
        ).withName("ExtendClimber");
    }

    /** Retract elevator downward. Releases ratchet while running, engages on end. */
    public Command retractCommand() {
        return runEnd(
            () -> {
                releaseRatchet();
                elevatorMotor.set(-ClimberConstants.kClimbSpeed);
            },
            () -> {
                elevatorMotor.set(0);
                engageRatchet();
            }
        ).withName("RetractClimber");
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

    /** Stop motor and engage ratchet. */
    public Command stopCommand() {
        return runOnce(() -> {
            elevatorMotor.set(0);
            engageRatchet();
        }).withName("StopClimber");
    }

    // ==================== SECTION: PERIODIC ====================
    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();

        Logger.recordOutput("Climber/Position", elevatorMotor.getPosition());
        Logger.recordOutput("Climber/RatchetEngaged", isRatchetEngaged());

        Logger.recordOutput("Timing/ClimberSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
    }
}
