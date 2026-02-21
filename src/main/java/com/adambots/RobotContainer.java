// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import java.io.File;

import com.adambots.commands.ShootCommands;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Buttons;
import com.adambots.lib.vision.PhotonVision;
import com.adambots.lib.vision.VisionSystem;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 *
 * <p>This class uses Inversion of Control (IoC) - hardware devices are created in RobotMap
 * and passed to subsystems here. This allows for easier testing and hardware abstraction.
 */
@Logged
public class RobotContainer {
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final TurretSubsystem turret;

    public RobotContainer() {
        shooter = new ShooterSubsystem(RobotMap.shooterLeftMotor, RobotMap.shooterRightMotor);
        hopper = new HopperSubsystem(RobotMap.hopperMotor, RobotMap.hopperPieceSensor);
        turret = new TurretSubsystem(RobotMap.turretMotor); 
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}