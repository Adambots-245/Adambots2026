package com.adambots.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.adambots.Constants;
import com.adambots.Constants.HopperConstants;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseDistanceSensor;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Hopper subsystem with hopper + uptake motors and CANRange distance sensor for piece detection.
 * Hopper and uptake always run together, so they share a single subsystem to prevent scheduling conflicts.
 */
public class HopperSubsystem extends SubsystemBase {

    private final BaseMotor hopperMotor;
    private final BaseMotor uptakeMotor;
    private final BaseDistanceSensor hopperPieceSensor;

    // Tunable fields (hot-reloaded from Shuffleboard when TUNING_ENABLED)
    private double hopperSpeed = HopperConstants.kHopperSpeed;
    private double uptakeSpeed = HopperConstants.kUptakeSpeed;
    private double detectionRange = HopperConstants.kDetectionRange;

    // Jam detection state
    private boolean reversing = false;
    private boolean wasFeedingLastCycle = false;
    private final Timer reverseTimer = new Timer();

    public HopperSubsystem(BaseMotor hopperMotor, BaseMotor uptakeMotor, BaseDistanceSensor hopperPieceSensor) {
        this.hopperMotor = hopperMotor;
        this.uptakeMotor = uptakeMotor;
        this.hopperPieceSensor = hopperPieceSensor;

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

    public boolean hasPiece() {
        return hopperPieceSensor.getDistance().in(Centimeters) <= detectionRange;
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
        boolean currentlyFeeding = hopperMotor.getOutputPercent() > 0;

        if (reversing) {
            if (reverseTimer.hasElapsed(HopperConstants.kJamReverseDuration)) {
                reversing = false;
                reverseTimer.restart(); // reuse as grace timer
            }
            wasFeedingLastCycle = false;
            return;
        }

        if (currentlyFeeding && !wasFeedingLastCycle) {
            reverseTimer.restart(); // feed just started — begin grace period
        }

        if (currentlyFeeding
                && reverseTimer.hasElapsed(HopperConstants.kJamGracePeriod)
                && hopperMotor.getVelocity().in(RotationsPerSecond) < HopperConstants.kJamVelocityThreshold) {
            reversing = true;
            reverseTimer.restart();
            hopperMotor.set(-hopperSpeed);
            uptakeMotor.set(-uptakeSpeed);
        }

        wasFeedingLastCycle = currentlyFeeding;
    }

    // ==================== Triggers ====================

    public final Trigger hasPieceTrigger = new Trigger(this::hasPiece);

    public Trigger isEmptyTrigger() {
        return new Trigger(() -> !hasPiece());
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

    public void setDetectionRange(double range) {
        detectionRange = range;
    }
}
