// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import com.adambots.lib.sensors.BaseDistanceSensor;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for BaseDistanceSensor interface.
 * Delegates to AdambotsLib's generated logger.
 */
@CustomLoggerFor(BaseDistanceSensor.class)
public class BaseDistanceSensorLogger extends ClassSpecificLogger<BaseDistanceSensor> {

    private static final com.adambots.lib.sensors.BaseDistanceSensorLogger delegate =
        new com.adambots.lib.sensors.BaseDistanceSensorLogger();

    public BaseDistanceSensorLogger() {
        super(BaseDistanceSensor.class);
    }

    @Override
    public void update(EpilogueBackend backend, BaseDistanceSensor sensor) {
        delegate.update(backend, sensor);
    }
}
