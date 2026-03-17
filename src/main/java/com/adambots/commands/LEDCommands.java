package com.adambots.commands;

import com.adambots.Constants;
import com.adambots.lib.subsystems.CANdleSubsystem;
import com.adambots.utils.HubActivation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for hub-activation LED feedback via the CANdle subsystem.
 *
 * <p>Behavior:
 * <ul>
 *   <li><strong>Hub active:</strong> Solid green</li>
 *   <li><strong>Hub inactive (countdown):</strong> Alliance color (red/blue) draining
 *       from top — LEDs turn off as time counts down</li>
 *   <li><strong>5 seconds before shift change:</strong> All LEDs strobe in alliance color</li>
 * </ul>
 */
public final class LEDCommands {

    private LEDCommands() {}

    /** Returns alliance color (red or blue). Defaults to red if unknown. */
    private static Color getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return Color.kBlue;
        }
        return Color.kRed;
    }

    /**
     * Default command: solid green when hub is active, alliance-color countdown
     * bar (draining from top) when inactive, strobe when 5 seconds remain.
     */
    public static Command hubStateCommand(CANdleSubsystem leds) {
        return Commands.run(() -> {
            int startIdx = Constants.kProgressStartIndex;
            int numLEDs = Constants.kProgressLEDCount;

            if (HubActivation.isOurHubActive()) {
                // Hub active — solid green
                leds.setColor(0, 255, 0);
            } else {
                double secondsLeft = HubActivation.secondsUntilOurHubActivates();

                if (secondsLeft <= 5.0) {
                    // 5 seconds left — strobe all LEDs in alliance color
                    Color c = getAllianceColor();
                    int r = (int) (c.red * 255);
                    int g = (int) (c.green * 255);
                    int b = (int) (c.blue * 255);
                    // Fast blink: alternate on/off every 5 cycles (100ms at 50Hz)
                    boolean on = (System.currentTimeMillis() / 100) % 2 == 0;
                    if (on) {
                        leds.setLEDs(r, g, b, startIdx, numLEDs);
                    } else {
                        leds.setLEDs(0, 0, 0, startIdx, numLEDs);
                    }
                } else {
                    // Countdown — alliance color draining from top
                    double fraction = MathUtil.clamp(
                        HubActivation.fractionUntilOurHubActivates(), 0.0, 1.0);
                    Color c = getAllianceColor();
                    int r = (int) (c.red * 255);
                    int g = (int) (c.green * 255);
                    int b = (int) (c.blue * 255);

                    // fraction=1.0 means just went inactive (all lit), 0.0 means about to activate (none lit)
                    int litCount = (int) ((1.0 - fraction) * numLEDs);

                    // Lit LEDs at the bottom (start of strip), dark at the top
                    if (litCount > 0) {
                        leds.setLEDs(r, g, b, startIdx, litCount);
                    }
                    int darkCount = numLEDs - litCount;
                    if (darkCount > 0) {
                        leds.setLEDs(0, 0, 0, startIdx + litCount, darkCount);
                    }
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
}
