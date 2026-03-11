package com.adambots.commands;

import com.adambots.Constants;
import com.adambots.lib.Constants.LEDConstants;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.utils.HubActivation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for hub-activation LED feedback via the CANdle subsystem.
 */
public final class LEDCommands {

    private LEDCommands() {}

    // Adambots yellow as 0-255 RGB for setLEDs calls
    private static final int AY_R = (int) (LEDConstants.adambotsYellow.red * 255);
    private static final int AY_G = (int) (LEDConstants.adambotsYellow.green * 255);
    private static final int AY_B = (int) (LEDConstants.adambotsYellow.blue * 255);

    /**
     * Default command: solid green when our hub is active, Adambots yellow
     * progress bar (draining toward activation) when inactive.
     */
    public static Command hubStateCommand(CANdleSubsystem leds) {
        return Commands.run(() -> {
            if (HubActivation.isOurHubActive()) {
                leds.setColor(0, 255, 0);
            } else {
                double fraction = MathUtil.clamp(
                    HubActivation.fractionUntilOurHubActivates(), 0.0, 1.0);
                int startIdx = Constants.kProgressStartIndex;
                int numLEDs = Constants.kProgressLEDCount;
                int litCount = (int) (fraction * numLEDs);
                if (litCount > 0) {
                    leds.setLEDs(AY_R, AY_G, AY_B, startIdx, litCount);
                }
                int remaining = numLEDs - litCount;
                if (remaining > 0) {
                    leds.setLEDs(0, 0, 0, startIdx + litCount, remaining);
                }
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
