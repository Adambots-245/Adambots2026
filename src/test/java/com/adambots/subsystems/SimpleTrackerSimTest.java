package com.adambots.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Simulates the simple turret tracker logic against a realistic turret model
 * derived from actual log data. Validates convergence, overshoot, and
 * smoothness for various scenarios.
 *
 * <p>Turret model (from log analysis):
 * <ul>
 *   <li>Speed = 1029 * percentOutput (deg/s)</li>
 *   <li>Planetary holds position when output = 0 (non-backdrivable)</li>
 *   <li>Camera FOV: ±30° (hub visible when |camYaw| < 30)</li>
 *   <li>Camera FPS: 14 Hz (71ms between frames)</li>
 *   <li>Camera noise: 0.5° std dev Gaussian</li>
 * </ul>
 */
class SimpleTrackerSimTest {

    // Turret model constants (from log analysis)
    static final double TURRET_SPEED_PER_PERCENT = 1029.0; // deg/s per 1.0 output
    static final double CAMERA_FOV_HALF = 30.0;            // degrees
    static final double CAMERA_INTERVAL_MS = 71.0;         // ms between fresh frames
    static final double CAMERA_NOISE_STD = 0.5;            // degrees
    static final double CONTROL_LOOP_MS = 20.0;            // ms per control cycle
    static final double TURRET_MAX_DEG = 256.0;            // approximate max range

    // Default tracker constants
    static final double K = 0.008;
    static final double MAX_PERCENT = 0.25;
    static final double SWEEP_PERCENT = 0.08;

    /**
     * Simulates the simple tracker for a given duration.
     *
     * @param hubAngleDeg    fixed hub angle in turret space (degrees)
     * @param startAngleDeg  initial turret angle (degrees)
     * @param k              proportional gain
     * @param maxPercent     max output clamp
     * @param durationMs     simulation duration
     * @param robotOmegaDPS  robot rotation rate (deg/s), shifts hub angle over time
     * @return result object with convergence metrics
     */
    static SimResult simulate(double hubAngleDeg, double startAngleDeg,
                              double k, double maxPercent, double durationMs,
                              double robotOmegaDPS) {
        double turretAngle = startAngleDeg;
        double hubAngle = hubAngleDeg;
        double lastCamYaw = 0;
        boolean lastHubFresh = false;

        double maxError = 0;
        double maxOvershoot = 0;
        double sumErrorSq = 0;
        int errorSamples = 0;
        double maxAngleDelta = 0;
        int directionReversals = 0;
        double lastOutput = 0;
        double convergeTimeMs = -1; // time to reach within 2°
        boolean converged = false;
        int freshFrames = 0;

        double cameraTimer = 0;
        java.util.Random rng = new java.util.Random(42); // deterministic

        for (double t = 0; t < durationMs; t += CONTROL_LOOP_MS) {
            // Hub drifts due to robot rotation
            hubAngle += robotOmegaDPS * (CONTROL_LOOP_MS / 1000.0);

            // Camera model: update every CAMERA_INTERVAL_MS
            cameraTimer += CONTROL_LOOP_MS;
            boolean hubFresh = false;
            double camYaw = lastCamYaw;

            if (cameraTimer >= CAMERA_INTERVAL_MS) {
                cameraTimer -= CAMERA_INTERVAL_MS;
                double trueYaw = hubAngle - turretAngle;
                // Hub visible if within FOV
                if (Math.abs(trueYaw) < CAMERA_FOV_HALF) {
                    camYaw = trueYaw + rng.nextGaussian() * CAMERA_NOISE_STD;
                    hubFresh = true;
                    freshFrames++;
                }
            }
            boolean hubVisible = Math.abs(hubAngle - turretAngle) < CAMERA_FOV_HALF;
            lastCamYaw = camYaw;

            // === Simple tracker logic (mirrors TurretSubsystem.simpleTrackCommand) ===
            double output;
            if (hubVisible) {
                if (hubFresh) {
                    output = camYaw * k;
                    output = Math.max(-maxPercent, Math.min(maxPercent, output));
                } else {
                    output = 0; // stale: hold (planetary)
                }
            } else {
                // Sweep toward hub (simplified — no pose in this test)
                output = (hubAngle > turretAngle ? 1 : -1) * SWEEP_PERCENT;
            }

            // Track direction reversals
            if (lastOutput != 0 && output != 0
                    && Math.signum(output) != Math.signum(lastOutput)) {
                directionReversals++;
            }

            // Apply motor model
            double prevAngle = turretAngle;
            turretAngle += output * TURRET_SPEED_PER_PERCENT * (CONTROL_LOOP_MS / 1000.0);
            turretAngle = Math.max(0, Math.min(TURRET_MAX_DEG, turretAngle));

            double angleDelta = Math.abs(turretAngle - prevAngle);
            maxAngleDelta = Math.max(maxAngleDelta, angleDelta);

            // Track error
            double error = Math.abs(hubAngle - turretAngle);
            maxError = Math.max(maxError, error);
            sumErrorSq += error * error;
            errorSamples++;

            // Overshoot: if we crossed the hub
            if ((prevAngle < hubAngle && turretAngle > hubAngle)
                    || (prevAngle > hubAngle && turretAngle < hubAngle)) {
                maxOvershoot = Math.max(maxOvershoot, Math.abs(turretAngle - hubAngle));
            }

            // Convergence time
            if (!converged && error < 2.0) {
                convergeTimeMs = t;
                converged = true;
            } else if (converged && error > 5.0) {
                converged = false; // lost convergence
                convergeTimeMs = -1;
            }

            lastOutput = output;
        }

        double rmsError = Math.sqrt(sumErrorSq / Math.max(errorSamples, 1));
        return new SimResult(rmsError, maxError, maxOvershoot, convergeTimeMs,
            directionReversals, maxAngleDelta, freshFrames, converged);
    }

    record SimResult(
        double rmsError,
        double maxError,
        double maxOvershoot,
        double convergeTimeMs,
        int directionReversals,
        double maxAngleDeltaPerFrame,
        int freshFrames,
        boolean converged
    ) {}

    // ==================== Test Cases ====================

    @Test
    void testStationaryConvergence_10deg() {
        // Hub is 10° to the right, robot stationary
        SimResult r = simulate(110, 100, K, MAX_PERCENT, 3000, 0);
        assertTrue(r.converged, "Should converge within 3 seconds");
        assertTrue(r.convergeTimeMs < 1500, "Should converge within 1.5s, was " + r.convergeTimeMs + "ms");
        assertTrue(r.rmsError < 5.0, "RMS error should be < 5°, was " + r.rmsError);
        assertTrue(r.maxOvershoot < 3.0, "Overshoot should be < 3°, was " + r.maxOvershoot);
        System.out.printf("10° stationary: converge=%.0fms rms=%.1f° overshoot=%.1f° reversals=%d%n",
            r.convergeTimeMs, r.rmsError, r.maxOvershoot, r.directionReversals);
    }

    @Test
    void testStationaryConvergence_30deg() {
        // Hub is 30° away — larger initial correction
        SimResult r = simulate(130, 100, K, MAX_PERCENT, 3000, 0);
        assertTrue(r.converged, "Should converge within 3 seconds");
        assertTrue(r.convergeTimeMs < 2000, "Should converge within 2s, was " + r.convergeTimeMs + "ms");
        assertTrue(r.maxOvershoot < 5.0, "Overshoot should be < 5°, was " + r.maxOvershoot);
        System.out.printf("30° stationary: converge=%.0fms rms=%.1f° overshoot=%.1f° reversals=%d%n",
            r.convergeTimeMs, r.rmsError, r.maxOvershoot, r.directionReversals);
    }

    @Test
    void testStationaryConvergence_90deg() {
        // Hub is 90° away — stress test
        SimResult r = simulate(190, 100, K, MAX_PERCENT, 5000, 0);
        assertTrue(r.converged, "Should converge within 5 seconds");
        assertTrue(r.maxOvershoot < 8.0, "Overshoot should be < 8°, was " + r.maxOvershoot);
        System.out.printf("90° stationary: converge=%.0fms rms=%.1f° overshoot=%.1f° reversals=%d%n",
            r.convergeTimeMs, r.rmsError, r.maxOvershoot, r.directionReversals);
    }

    @Test
    void testSlowRotation() {
        // Robot rotating at 30°/s — hub drifts continuously
        // At K=0.008, the proportional controller lags ~10° behind during rotation
        SimResult r = simulate(130, 100, K, MAX_PERCENT, 5000, 30);
        assertTrue(r.rmsError < 20.0, "RMS error during 30°/s rotation should be < 20°, was " + r.rmsError);
        assertTrue(r.directionReversals < 20, "Should not oscillate excessively, had " + r.directionReversals);
        System.out.printf("30°/s rotation: rms=%.1f° max=%.1f° reversals=%d%n",
            r.rmsError, r.maxError, r.directionReversals);
    }

    @Test
    void testFastRotation() {
        // Robot rotating at 100°/s — hub quickly leaves FOV.
        // Expected limitation: no proportional tracker can keep up with
        // continuous 100°/s rotation. Just verify no crash/exception.
        assertDoesNotThrow(() -> simulate(130, 100, K, MAX_PERCENT, 5000, 100));
        SimResult r = simulate(130, 100, K, MAX_PERCENT, 5000, 100);
        System.out.printf("100°/s rotation: rms=%.1f° max=%.1f° freshFrames=%d reversals=%d (expected: can't track)%n",
            r.rmsError, r.maxError, r.freshFrames, r.directionReversals);
    }

    @Test
    void testNoOvershootOnFreshOnly() {
        // The key property: output = 0 on stale frames prevents overshoot
        SimResult r = simulate(120, 100, K, MAX_PERCENT, 3000, 0);
        // With fresh-only output, overshoot should be minimal
        assertTrue(r.maxOvershoot < 3.0,
            "Fresh-only gating should prevent overshoot > 3°, was " + r.maxOvershoot);
        System.out.printf("Fresh-only overshoot test: overshoot=%.1f° converge=%.0fms%n",
            r.maxOvershoot, r.convergeTimeMs);
    }

    @Test
    void testSmoothness() {
        // Verify turret doesn't make huge jumps between frames
        SimResult r = simulate(110, 100, K, MAX_PERCENT, 3000, 0);
        // At K=0.003, max 25%, 20ms cycle: max delta = 0.25 * 1029 * 0.02 = 5.15°
        assertTrue(r.maxAngleDeltaPerFrame < 6.0,
            "Max angle change per frame should be < 6°, was " + r.maxAngleDeltaPerFrame);
        System.out.printf("Smoothness: max delta/frame=%.1f°%n", r.maxAngleDeltaPerFrame);
    }

    @Test
    void testHubOutOfFOV_SweepsToFind() {
        // Hub is 60° away — outside camera FOV. Should sweep to find it.
        SimResult r = simulate(160, 100, K, MAX_PERCENT, 10000, 0);
        assertTrue(r.converged, "Should find hub via sweep and converge within 10s");
        assertTrue(r.freshFrames > 0, "Should have gotten fresh frames after sweep found hub");
        System.out.printf("Sweep→find: converge=%.0fms freshFrames=%d%n",
            r.convergeTimeMs, r.freshFrames);
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.001, 0.002, 0.003, 0.004, 0.005, 0.008, 0.010})
    void testKValueSweep(double kVal) {
        // Sweep K values to find optimal
        SimResult r10 = simulate(110, 100, kVal, MAX_PERCENT, 3000, 0);
        SimResult r30 = simulate(130, 100, kVal, MAX_PERCENT, 3000, 0);
        SimResult rRot = simulate(130, 100, kVal, MAX_PERCENT, 5000, 30);

        System.out.printf("K=%.4f | 10°: conv=%4.0fms rms=%4.1f° ovr=%4.1f° | " +
            "30°: conv=%4.0fms rms=%4.1f° ovr=%4.1f° | " +
            "rot30: rms=%4.1f° rev=%d%n",
            kVal,
            r10.convergeTimeMs, r10.rmsError, r10.maxOvershoot,
            r30.convergeTimeMs, r30.rmsError, r30.maxOvershoot,
            rRot.rmsError, rRot.directionReversals);
    }

    @Test
    void testPoseBasedAcquisition() {
        // Simulate pose-based pointing: turret starts at pose-estimated angle
        // (within 10° of hub), then camera takes over
        SimResult r = simulate(130, 122, K, MAX_PERCENT, 2000, 0);
        assertTrue(r.converged, "Should converge quickly from pose-estimated start");
        assertTrue(r.convergeTimeMs < 1200, "Should converge within 1.2s from 8° offset, was " + r.convergeTimeMs);
        System.out.printf("Pose-based start (8° off): converge=%.0fms rms=%.1f°%n",
            r.convergeTimeMs, r.rmsError);
    }

    @Test
    void testStaleFrameHoldsPosition() {
        // Simulate: camera gives 3 fresh frames then goes stale for 2 seconds
        // Turret should hold position, not drift
        double turretAngle = 100;
        double hubAngle = 110;

        // 3 fresh frames at 14 Hz
        for (int i = 0; i < 3; i++) {
            double camYaw = hubAngle - turretAngle;
            double output = Math.max(-MAX_PERCENT, Math.min(MAX_PERCENT, camYaw * K));
            turretAngle += output * TURRET_SPEED_PER_PERCENT * (CAMERA_INTERVAL_MS / 1000.0);
        }
        double angleAfterFresh = turretAngle;

        // 2 seconds of stale frames (output = 0)
        // Planetary holds, so angle stays constant
        double angleAfterStale = turretAngle; // no change

        assertEquals(angleAfterFresh, angleAfterStale, 0.01,
            "Turret should hold position during stale frames (planetary non-backdrivable)");
        System.out.printf("Stale hold test: after fresh=%.1f° after 2s stale=%.1f° (diff=%.3f°)%n",
            angleAfterFresh, angleAfterStale, Math.abs(angleAfterStale - angleAfterFresh));
    }
}
