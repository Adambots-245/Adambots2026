// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import com.adambots.lib.sensors.BaseGyro;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for BaseGyro interface.
 * Delegates to AdambotsLib's generated logger.
 */
@CustomLoggerFor(BaseGyro.class)
public class BaseGyroLogger extends ClassSpecificLogger<BaseGyro> {

    private static final com.adambots.lib.sensors.BaseGyroLogger delegate =
        new com.adambots.lib.sensors.BaseGyroLogger();

    public BaseGyroLogger() {
        super(BaseGyro.class);
    }

    @Override
    public void update(EpilogueBackend backend, BaseGyro gyro) {
        delegate.update(backend, gyro);
    }
}
