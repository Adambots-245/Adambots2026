// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import com.adambots.lib.actuators.BaseMotor;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for BaseMotor interface.
 *
 * <p>This is needed because Epilogue's annotation processor cannot discover
 * custom loggers from external libraries (AdambotsLib). This thin wrapper
 * delegates to AdambotsLib's generated BaseMotorLogger.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html">WPILib Epilogue Docs</a>
 */
@CustomLoggerFor(BaseMotor.class)
public class BaseMotorLogger extends ClassSpecificLogger<BaseMotor> {

    // Reuse a single instance of AdambotsLib's logger
    private static final com.adambots.lib.actuators.BaseMotorLogger delegate =
        new com.adambots.lib.actuators.BaseMotorLogger();

    public BaseMotorLogger() {
        super(BaseMotor.class);
    }

    @Override
    public void update(EpilogueBackend backend, BaseMotor motor) {
        delegate.update(backend, motor);
    }
}
