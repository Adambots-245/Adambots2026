// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.ShooterTestConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.BaseMotor.ControlMode;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

// ==================== Tunable Setup ====================

    /**
     * Registers turret tunable GenericEntry fields on the current Dash tab.
     * Call after Dash.useTab() in RobotContainer.
     * @param pos position tracker {col, row}, updated in place
     * @param cols max columns per row before wrapping
     */
    public void setupturretTunables(int[] pos, int cols) {
        turretPEntry = Dash.addTunable("turret kP", ShooterTestConstants.kTurretP, pos[0], pos[1]);
        advance(pos, cols);
        turretIEntry = Dash.addTunable("turret kI", ShooterTestConstants.kTurretI, pos[0], pos[1]);
        advance(pos, cols);
        turretDEntry = Dash.addTunable("turret kD", ShooterTestConstants.kTurretD, pos[0], pos[1]);
        advance(pos, cols);
    

        // Table distances in one row, RPS values directly below
    }

   /** Advances the position tracker to the next column, wrapping to the next row if needed. */
    private static void advance(int[] pos, int cols) {
        pos[0]++;
        if (pos[0] >= cols) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    /** Jumps to column 0 of the next row (no-op if already at column 0). */
    private static void newRow(int[] pos) {
        if (pos[0] != 0) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    public void setTurretAngle(double degrees){
      degrees = MathUtil.clamp(degrees,
        ShooterTestConstants.kTurretMinDegrees, ShooterTestConstants.kTurretMaxDegrees);
      double rotations = (degrees/360.0) * ShooterTestConstants.kTurretGearRatio;
      turretMotor.set(ControlMode.POSITION, rotations);
    }

    public void StopTurret(){
      turretMotor.set(0);
    }

    public double getTurretAngleDegrees(){
      return (turretMotor.getPosition() / ShooterTestConstants.kTurretGearRatio) * 360.0;
    }

  @Override
    public void periodic() {
        // Read tunable PID values and apply only when changed
        if (turretPEntry != null) {
            double p = turretPEntry.getDouble(ShooterTestConstants.kTurretP);
            double i = turretIEntry.getDouble(ShooterTestConstants.kTurretI);
            double d = turretDEntry.getDouble(ShooterTestConstants.kTurretD);

            if (p != lastTurretP || i != lastTurretI || d != lastTurretD) {
                turretMotor.setPID(0, p, i, d, ShooterTestConstants.kTurretFF);
                lastTurretP = p;
                lastTurretI = i;
                lastTurretD = d;
            }
        }
    }




    // ==================== Command Factories ====================

    public Command aimTurretCommand(double degrees){
      return Commands.runOnce(() -> setTurretAngle(degrees))
        .withName("Turret " + degrees + " deg");
    }
    //use for/delete after tuning
    public Command aimTurretManualCommand(){
      return Commands.run(() -> setTurretAngle(
            turretAngleEntry != null ? turretAngleEntry.getDouble(90.0) : 90.0))
        .withName("Aim Turret Manual"); 
    }

    public Command stopTurret(){
      return Commands.runOnce(this::stopTurret)
        .withName("Stop Turret");
    }


}
