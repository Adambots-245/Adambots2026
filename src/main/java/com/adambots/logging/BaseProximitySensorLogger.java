// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import com.adambots.lib.sensors.BaseProximitySensor;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for BaseProximitySensor interface.
 * Delegates to AdambotsLib's generated logger.
 */
@CustomLoggerFor(BaseProximitySensor.class)
public class BaseProximitySensorLogger extends ClassSpecificLogger<BaseProximitySensor> {

    private static final com.adambots.lib.sensors.BaseProximitySensorLogger delegate =
        new com.adambots.lib.sensors.BaseProximitySensorLogger();

    public BaseProximitySensorLogger() {
        super(BaseProximitySensor.class);
    }

    @Override
    public void update(EpilogueBackend backend, BaseProximitySensor sensor) {
        delegate.update(backend, sensor);
    }
}
