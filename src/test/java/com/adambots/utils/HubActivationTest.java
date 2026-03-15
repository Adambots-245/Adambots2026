package com.adambots.utils;

import static org.junit.jupiter.api.Assertions.*;

import com.adambots.utils.HubActivation.Shift;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class HubActivationTest {

    // --- shiftForTime tests ---

    @ParameterizedTest
    @CsvSource({
        "135.0, TRANSITION",
        "131.0, TRANSITION",
        "130.1, TRANSITION",
        "130.0, SHIFT_1",
        "120.0, SHIFT_1",
        "105.1, SHIFT_1",
        "105.0, SHIFT_2",
        "90.0,  SHIFT_2",
        "80.1,  SHIFT_2",
        "80.0,  SHIFT_3",
        "65.0,  SHIFT_3",
        "55.1,  SHIFT_3",
        "55.0,  SHIFT_4",
        "40.0,  SHIFT_4",
        "30.1,  SHIFT_4",
        "30.0,  ENDGAME",
        "15.0,  ENDGAME",
        "0.0,   ENDGAME",
    })
    void shiftForTime_returnsCorrectShift(double time, Shift expected) {
        assertEquals(expected, HubActivation.shiftForTime(time));
    }

    @Test
    void shiftForTime_negativeTime_returnsTransition() {
        assertEquals(Shift.TRANSITION, HubActivation.shiftForTime(-1.0));
    }

    // --- secondsUntilNextShiftChange tests ---

    @Test
    void secondsUntilNextShiftChange_inTransition() {
        assertEquals(3.0, HubActivation.secondsUntilNextShiftChangeForTime(133.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_inShift1() {
        assertEquals(15.0, HubActivation.secondsUntilNextShiftChangeForTime(120.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_inShift2() {
        assertEquals(10.0, HubActivation.secondsUntilNextShiftChangeForTime(90.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_inShift3() {
        assertEquals(5.0, HubActivation.secondsUntilNextShiftChangeForTime(60.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_inShift4() {
        assertEquals(10.0, HubActivation.secondsUntilNextShiftChangeForTime(40.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_inEndgame_returnsZero() {
        assertEquals(0.0, HubActivation.secondsUntilNextShiftChangeForTime(20.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_notInMatch_returnsZero() {
        assertEquals(0.0, HubActivation.secondsUntilNextShiftChangeForTime(-1.0), 0.01);
    }

    // --- Shift boundary edge cases ---

    @Test
    void shiftForTime_exactlyAtBoundary_130() {
        // 130.0 is the start of SHIFT_1 (time <= 130 enters SHIFT_1)
        assertEquals(Shift.SHIFT_1, HubActivation.shiftForTime(130.0));
    }

    @Test
    void shiftForTime_justAboveBoundary_130() {
        assertEquals(Shift.TRANSITION, HubActivation.shiftForTime(130.01));
    }

    // --- secondsUntilActiveForTime tests (these test the internal logic without DriverStation) ---
    // Note: isOurHubActiveForTime and secondsUntilActiveForTime depend on DriverStation
    // for alliance/game data, so full integration testing requires a robot environment.
    // Here we test the time-based logic that doesn't need mocking.

    @Test
    void shiftForTime_allBoundariesCoveredCorrectly() {
        // Verify no gaps in shift coverage
        for (double t = 0.0; t <= 135.0; t += 0.5) {
            Shift shift = HubActivation.shiftForTime(t);
            assertNotNull(shift, "Shift should not be null at time " + t);
        }
    }

    @Test
    void secondsUntilNextShiftChange_atExactBoundary() {
        // At exactly 105.0, we're in SHIFT_2, so time until next change = 105 - 80 = 25
        assertEquals(25.0, HubActivation.secondsUntilNextShiftChangeForTime(105.0), 0.01);
    }

    @Test
    void secondsUntilNextShiftChange_atExactShiftEnd() {
        // At 80.0 we just entered SHIFT_3, time until next change = 80 - 55 = 25
        assertEquals(25.0, HubActivation.secondsUntilNextShiftChangeForTime(80.0), 0.01);
    }
}
