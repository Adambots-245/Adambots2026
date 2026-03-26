package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.adambots.Constants;
import com.adambots.Constants.HopperConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hopper subsystem with hopper + uptake motors.
 * Hopper and uptake always run together, so they share a single subsystem to prevent scheduling conflicts.
 */
public class HopperSubsystem extends SubsystemBase {

    private final BaseMotor hopperMotor;
    private final BaseMotor uptakeMotor;

    // Tunable fields (hot-reloaded from Shuffleboard when TUNING_ENABLED)
    private double hopperSpeed = HopperConstants.kHopperSpeed;
    private double uptakeSpeed = HopperConstants.kUptakeSpeed;

    // Jam detection state
    private boolean reversing = false;
    private boolean wasFeedingLastCycle = false;
    private final Timer reverseTimer = new Timer();

    public HopperSubsystem(BaseMotor hopperMotor, BaseMotor uptakeMotor) {
        this.hopperMotor = hopperMotor;
        this.uptakeMotor = uptakeMotor;

        hopperMotor.configure()
            .brakeMode(true)
            .currentLimits(70, HopperConstants.kHopperSupplyCurrentLimit, 3000)
            .apply();

        uptakeMotor.configure()
            .brakeMode(true)
            .inverted(true)
            .currentLimits(60, HopperConstants.kUptakeSupplyCurrentLimit, 3000)
            .apply();

        if (Constants.HOPPER_TAB) {
            setupDash();
        }
    }


    private void feed() {
        if (!reversing) {
            hopperMotor.set(hopperSpeed);
            uptakeMotor.set(uptakeSpeed);
        }
    }

    private void reverse() {
        hopperMotor.set(-hopperSpeed);
        uptakeMotor.set(-uptakeSpeed);
    }

    private void stop() {
        hopperMotor.set(0);
        uptakeMotor.set(0);
        reversing = false;
    }

    @Override
    public void periodic() {
        // ==================== Hopper jam detection ====================
        //
        // State fields:
        //   reversing              — true while motors run backward to clear a jam
        //   wasFeedingLastCycle    — tracks rising edge (stopped → feeding) to start grace period
        //   reverseTimer           — reused for both reverse duration and grace period timing
        //
        // Normal sequence:
        //   1. feedCommand() starts → feed() sets both motors forward each cycle
        //   2. periodic() detects rising edge → restarts timer (grace period begins)
        //   3. After grace period (0.25s): motors have spun up, jam check activates
        //   4. If hopper velocity stays above threshold → no jam, feeding continues
        //   5. Command ends → stop() sets motors to 0 and clears reversing
        //
        // Jam sequence:
        //   1. Ball jams hopper → hopper motor velocity drops below threshold (0.5 RPS)
        //   2. periodic() detects jam → sets reversing=true, calls reverse()
        //   3. feed() sees reversing → skips forward set → reverse continues
        //   4. After reverse duration (0.5s) → reversing=false, timer restarts (grace)
        //   5. Next feed() call resumes forward → grace period prevents immediate re-trigger
        //   6. If jam persists → cycle repeats (buzzes back and forth as operator feedback)
        //
        boolean currentlyFeeding = hopperMotor.getOutputPercent() > 0;

        // --- REVERSING state: motors are running backward to clear a jam ---
        if (reversing) {
            if (reverseTimer.hasElapsed(HopperConstants.kJamReverseDuration)) {
                // Reverse complete — return to feeding on next cycle.
                // Restart timer as grace period so jam detection doesn't
                // trigger immediately while motors spin back up.
                reversing = false;
                reverseTimer.restart();
            }
            wasFeedingLastCycle = false;
            return;
        }

        // --- FEEDING state: detect rising edge to start grace period ---
        if (currentlyFeeding && !wasFeedingLastCycle) {
            // Feed just started (transition from stopped/reversed to forward).
            // Begin grace period — give motors time to spin up before
            // checking for jams, otherwise low initial velocity triggers
            // a false jam detection.
            reverseTimer.restart();
        }

        // --- JAM CHECK: only after grace period has elapsed ---
        if (currentlyFeeding
                && reverseTimer.hasElapsed(HopperConstants.kJamGracePeriod)
                && hopperMotor.getVelocity().in(RotationsPerSecond) < HopperConstants.kJamVelocityThreshold) {
            // Agitator velocity dropped below threshold — jam detected.
            // Reverse both motors to clear the blockage.
            reversing = true;
            reverseTimer.restart();
            reverse();
        }

        wasFeedingLastCycle = currentlyFeeding;
    }

    // ==================== Command Factories ====================

    public Command feedCommand() {
        return runEnd(this::feed, this::stop)
            .withName("Feed");
    }

    public Command reverseCommand() {
        return runEnd(this::reverse, this::stop)
            .withName("Reverse Hopper");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Stop Hopper");
    }

    public void setupDash(){
        Dash.useTab("Hopper");

        int row = 0;
        int col = 0;
        Dash.addCommand("Feed", feedCommand(), col++, row);
        Dash.addCommand("Stop", stopCommand(), col++, row);

        Dash.useDefaultTab();
    }

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setHopperSpeed(double speed) {
        hopperSpeed = speed;
    }

    public void setUptakeSpeed(double speed) {
        uptakeSpeed = speed;
    }

}
