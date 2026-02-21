// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ShooterTestConstants;
import com.adambots.lib.actuators.BaseMotor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private GenericEntry turretPEntry;
  private GenericEntry turretIEntry;
  private GenericEntry turretDEntry;
  private GenericEntry turretAngleEntry;
  private double lastTurretP, lastTurretI, lastTurretD;
  
  private final BaseMotor turretMotor;

  public TurretSubsystem(BaseMotor turretMotor) {
    this.turretMotor = turretMotor;
    configureMotors();
  }

  private void configureMotors() {
      turretMotor.configure()
            .pid(ShooterTestConstants.kTurretP, ShooterTestConstants.kTurretI,
                 ShooterTestConstants.kTurretD, ShooterTestConstants.kTurretFF)
            .brakeMode(true)   // hold position
            .currentLimits(ShooterTestConstants.kTurretStallCurrentLimit,
                           ShooterTestConstants.kTurretFreeCurrentLimit, 3000)
            .apply();
        turretMotor.configureSoftLimits(
            ShooterTestConstants.kTurretForwardLimit,
            ShooterTestConstants.kTurretReverseLimit, true);
        turretMotor.setPosition(0);

        lastTurretP = ShooterTestConstants.kTurretP;
        lastTurretI = ShooterTestConstants.kTurretI;
        lastTurretD = ShooterTestConstants.kTurretD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
