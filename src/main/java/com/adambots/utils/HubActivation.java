package com.adambots.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Static utility for tracking hub activation state during teleop in the 2026 REBUILT game.
 *
 * The FMS sends a single character ('R' or 'B') indicating which alliance's hub goes
 * inactive first. That alliance is active during Shifts 2 and 4; the other alliance is
 * active during Shifts 1 and 3. Both alliances are active during the transition period
 * and endgame.
 *
 * <p><strong>Test Mode:</strong> Call {@link #initTestMode()} at startup to add dashboard
 * controls for testing without an FMS. Toggle "Hub/TestMode" on, set "Hub/TestGameData"
 * to "R" or "B", and adjust "Hub/TestMatchTime" (135→0) to simulate shifts.
 */
public final class HubActivation {

    private HubActivation() {}

    // --- Test mode state ---
    private static boolean testModeInitialized = false;

    /** Shift duration in seconds. */
    static final double SHIFT_DURATION = 25.0;

    /** Match time remaining at the end of each period (counting down from 135s). */
    static final double TRANSITION_END = 130.0;
    static final double SHIFT_1_END = 105.0;
    static final double SHIFT_2_END = 80.0;
    static final double SHIFT_3_END = 55.0;
    static final double SHIFT_4_END = 30.0;

    public enum Shift {
        TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME
    }

    /**
     * Publishes test mode controls to SmartDashboard for testing hub activation
     * without an FMS. Call once during robot initialization.
     */
    public static void initTestMode() {
        SmartDashboard.putBoolean("Hub/TestMode", false);
        SmartDashboard.putString("Hub/TestGameData", "R");
        SmartDashboard.putNumber("Hub/TestMatchTime", 135.0);
        testModeInitialized = true;
    }

    /** Returns match time from test mode dashboard or DriverStation. */
    private static double getMatchTime() {
        if (testModeInitialized && SmartDashboard.getBoolean("Hub/TestMode", false)) {
            return SmartDashboard.getNumber("Hub/TestMatchTime", 135.0);
        }
        return DriverStation.getMatchTime();
    }

    /** Returns game-specific message from test mode dashboard or DriverStation. */
    private static String getGameData() {
        if (testModeInitialized && SmartDashboard.getBoolean("Hub/TestMode", false)) {
            return SmartDashboard.getString("Hub/TestGameData", "R");
        }
        return DriverStation.getGameSpecificMessage();
    }

    /**
     * Determines the current shift based on match time remaining.
     * Match time counts down from 135 in teleop.
     */
    public static Shift getCurrentShift() {
        return shiftForTime(getMatchTime());
    }

    /**
     * Returns true if our alliance's hub is currently active.
     * Defaults to true if game data is not yet available or we're not in a match.
     */
    public static boolean isOurHubActive() {
        return isOurHubActiveForTime(getMatchTime());
    }

    /**
     * Returns the number of seconds until our hub becomes active.
     * Returns 0.0 if our hub is already active.
     */
    public static double secondsUntilOurHubActivates() {
        return secondsUntilActiveForTime(getMatchTime());
    }

    /**
     * Returns a fraction (0.0 to 1.0) representing how far we are through the current
     * inactive shift. 1.0 = just went inactive, 0.0 = about to become active.
     * Returns 0.0 if our hub is already active.
     *
     * Useful for LED animations: {@code int ledsOff = (int)(fractionUntilOurHubActivates() * totalLeds);}
     */
    public static double fractionUntilOurHubActivates() {
        return secondsUntilOurHubActivates() / SHIFT_DURATION;
    }

    /**
     * Returns the number of seconds until the next shift change.
     * Returns 0.0 if not in a match or during endgame.
     */
    public static double secondsUntilNextShiftChange() {
        return secondsUntilNextShiftChangeForTime(getMatchTime());
    }

    // --- Trigger factories ---

    /** Trigger that is true when our hub is active. */
    public static Trigger ourHubActiveTrigger() {
        return new Trigger(HubActivation::isOurHubActive);
    }

    /** Trigger that is true when our hub is inactive. */
    public static Trigger ourHubInactiveTrigger() {
        return new Trigger(() -> !isOurHubActive());
    }

    /** Trigger that is true during the endgame period (last 30 seconds). */
    public static Trigger endGameTrigger() {
        return new Trigger(() -> getCurrentShift() == Shift.ENDGAME);
    }

    /**
     * Trigger that is true when a shift change is within the given warning window.
     * @param warningSeconds seconds before shift change to trigger
     */
    public static Trigger shiftChangeSoonTrigger(double warningSeconds) {
        return new Trigger(() -> {
            double remaining = secondsUntilNextShiftChange();
            return remaining > 0 && remaining <= warningSeconds;
        });
    }

    // --- Internal logic (package-private for testing) ---

    static Shift shiftForTime(double matchTime) {
        if (matchTime < 0) {
            return Shift.TRANSITION; // not in a match
        }
        if (matchTime > TRANSITION_END) {
            return Shift.TRANSITION;
        } else if (matchTime > SHIFT_1_END) {
            return Shift.SHIFT_1;
        } else if (matchTime > SHIFT_2_END) {
            return Shift.SHIFT_2;
        } else if (matchTime > SHIFT_3_END) {
            return Shift.SHIFT_3;
        } else if (matchTime > SHIFT_4_END) {
            return Shift.SHIFT_4;
        } else {
            return Shift.ENDGAME;
        }
    }

    /**
     * Returns true if the given alliance character matches our alliance.
     * 'R' matches Red, 'B' matches Blue.
     */
    private static Boolean weAreInactiveFirst() {
        String gameData = getGameData();
        if (gameData == null || gameData.isEmpty()) {
            return null; // not yet received
        }

        char inactiveFirstChar = gameData.charAt(0);
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return null; // alliance unknown
        }

        Alliance alliance = allianceOpt.get();
        if (inactiveFirstChar == 'R') {
            return alliance == Alliance.Red;
        } else if (inactiveFirstChar == 'B') {
            return alliance == Alliance.Blue;
        }
        return null; // unexpected character
    }

    static boolean isOurHubActiveForTime(double matchTime) {
        if (matchTime < 0) {
            return true; // not in a match, assume active
        }

        Shift shift = shiftForTime(matchTime);

        if (shift == Shift.TRANSITION || shift == Shift.ENDGAME) {
            return true; // both alliances active
        }

        Boolean inactiveFirst = weAreInactiveFirst();
        if (inactiveFirst == null) {
            return true; // unknown, default to active
        }

        if (inactiveFirst) {
            // We go inactive first → active in Shifts 2 and 4
            return shift == Shift.SHIFT_2 || shift == Shift.SHIFT_4;
        } else {
            // We go inactive second → active in Shifts 1 and 3
            return shift == Shift.SHIFT_1 || shift == Shift.SHIFT_3;
        }
    }

    static double secondsUntilActiveForTime(double matchTime) {
        if (isOurHubActiveForTime(matchTime)) {
            return 0.0;
        }

        // We're in an inactive shift. Find when this shift ends (= our next active shift starts).
        Shift shift = shiftForTime(matchTime);
        double shiftEnd;
        switch (shift) {
            case SHIFT_1: shiftEnd = SHIFT_1_END; break;
            case SHIFT_2: shiftEnd = SHIFT_2_END; break;
            case SHIFT_3: shiftEnd = SHIFT_3_END; break;
            case SHIFT_4: shiftEnd = SHIFT_4_END; break;
            default: return 0.0; // transition/endgame are always active
        }
        return matchTime - shiftEnd;
    }

    static double secondsUntilNextShiftChangeForTime(double matchTime) {
        if (matchTime < 0) {
            return 0.0;
        }

        Shift shift = shiftForTime(matchTime);
        switch (shift) {
            case TRANSITION: return matchTime - TRANSITION_END;
            case SHIFT_1: return matchTime - SHIFT_1_END;
            case SHIFT_2: return matchTime - SHIFT_2_END;
            case SHIFT_3: return matchTime - SHIFT_3_END;
            case SHIFT_4: return matchTime - SHIFT_4_END;
            case ENDGAME: return 0.0;
            default: return 0.0;
        }
    }
}
