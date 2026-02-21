// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.actuators.TalonFXMotor;
import com.adambots.lib.sensors.BaseProximitySensor;
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

    // ==================== CONTROLLER PORTS ====================
    /** Driver controller port - Logitech Extreme 3D Pro joystick */
    public static final int kDriverJoystickPort = 0;
    /** Operator controller port - Xbox controller */
    public static final int kOperatorXboxPort = 1;

    // ==================== VISION ====================
    // Camera names are configured in VisionConstants, not here
    // PhotonVision cameras are accessed by name, not port
}
