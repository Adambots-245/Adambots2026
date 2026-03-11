package com.adambots.commands;

import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.utils.HubActivation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for hub-activation LED feedback via the CANdle subsystem.
 */
public final class LEDCommands {

    private LEDCommands() {}

    /**
     * Default command: solid green when our hub is active, red→green color ramp
     * when inactive (based on fraction until activation).
     */
    public static Command hubStateCommand(CANdleSubsystem leds) {
        return Commands.run(() -> {
            if (HubActivation.isOurHubActive()) {
                leds.setColor(0, 255, 0);
            } else {
                double fraction = HubActivation.fractionUntilOurHubActivates();
                int r = (int) (fraction * 255);
                int g = (int) ((1.0 - fraction) * 255);
                leds.setColor(r, g, 0);
            }
        }, leds).withName("HubStateLEDs");
    }

    /**
     * Flash green 3 times when our hub becomes active.
     * Bind to {@code HubActivation.ourHubActiveTrigger().onTrue(...)}.
     */
    public static Command hubActivatedFlashCommand(CANdleSubsystem leds) {
        return leds.blinkCommand(Color.kGreen, 3).withName("HubActivatedFlash");
    }

    /**
     * Orange strobe for 1.5 seconds as a warning before a shift change.
     * Bind to {@code HubActivation.shiftChangeSoonTrigger(5.0).onTrue(...)}.
     */
    public static Command hubWarningCommand(CANdleSubsystem leds) {
        return leds.strobeCommand(Color.kOrange, 1.5).withName("HubWarningStrobe");
    }
}
