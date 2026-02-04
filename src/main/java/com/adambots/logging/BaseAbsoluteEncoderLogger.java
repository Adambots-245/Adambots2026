// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import com.adambots.lib.sensors.BaseAbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for BaseAbsoluteEncoder interface.
 * Delegates to AdambotsLib's generated logger.
 */
@CustomLoggerFor(BaseAbsoluteEncoder.class)
public class BaseAbsoluteEncoderLogger extends ClassSpecificLogger<BaseAbsoluteEncoder> {

    private static final com.adambots.lib.sensors.BaseAbsoluteEncoderLogger delegate =
        new com.adambots.lib.sensors.BaseAbsoluteEncoderLogger();

    public BaseAbsoluteEncoderLogger() {
        super(BaseAbsoluteEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, BaseAbsoluteEncoder encoder) {
        delegate.update(backend, encoder);
    }
}
