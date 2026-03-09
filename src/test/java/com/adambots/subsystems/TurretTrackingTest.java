package com.adambots.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;

import edu.wpi.first.math.MathUtil;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for TurretTracking pure-math helpers.
 * Tests angle selection, pose conversion, clamping, and lead angle computation.
 */
class TurretTrackingTest {

    // Default pose offset: 360 - kTurretForwardDegrees(90) = 270
    private static final double POSE_OFFSET = 360.0 - TurretConstants.kTurretForwardDegrees;
    private static final double MAX_DEG = TurretConstants.kTurretMaxDegrees; // 180
    private static final double TOLERANCE = 0.1;
    private static final double BALL_SPEED = TurretTrackingConstants.kBallExitSpeedMps; // 15.0

    // ==================== toAbsoluteTurretAngle ====================

    @Test
    void cameraTarget_returnsCurrentPlusOffset() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            90.0, 5.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(95.0, result, TOLERANCE);
    }

    @Test
    void cameraTarget_clampsToMax() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            175.0, 10.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(MAX_DEG, result, TOLERANCE);
    }

    @Test
    void cameraTarget_clampsToMin() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            5.0, -10.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void poseTarget_usedWhenCameraLost() {
        // Pose bearing 0° (hub directly ahead in robot frame)
        // With offset 270, turretCenter = 270 + 90 = 360
        // inputModulus(0 - 360, -180, 180) = 0 → result = 90
        double result = TurretTracking.toAbsoluteTurretAngle(
            90.0, 0.0, false, 0.0, true, POSE_OFFSET);
        assertEquals(90.0, result, TOLERANCE);
    }

    @Test
    void neitherAvailable_holdsCurrentAngle() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            45.0, 0.0, false, 0.0, false, POSE_OFFSET);
        assertEquals(45.0, result, TOLERANCE);
    }

    @Test
    void cameraTakesPriorityOverPose() {
        // Camera says +5°, pose says something else — camera wins
        double result = TurretTracking.toAbsoluteTurretAngle(
            90.0, 5.0, true, 180.0, true, POSE_OFFSET);
        assertEquals(95.0, result, TOLERANCE);
    }

    // ==================== poseAngleToTurretAngle ====================

    @Test
    void poseDirectlyAhead_turretCenter() {
        // Hub at 0° bearing (robot-relative "ahead")
        // turretCenter = 270 + 90 = 360; inputModulus(0 - 360, -180, 180) = 0; result = 90
        double result = TurretTracking.poseAngleToTurretAngle(0.0, POSE_OFFSET);
        assertEquals(90.0, result, TOLERANCE);
    }

    @Test
    void poseToRight_turretLow() {
        // Hub at -90° bearing (to the right)
        // inputModulus(-90 - 360, -180, 180) = inputModulus(-450, -180, 180) = -90
        // result = -90 + 90 = 0
        double result = TurretTracking.poseAngleToTurretAngle(-90.0, POSE_OFFSET);
        assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void poseToLeft_turretHigh() {
        // Hub at 90° bearing (to the left)
        // inputModulus(90 - 360, -180, 180) = inputModulus(-270, -180, 180) = 90
        // result = 90 + 90 = 180
        double result = TurretTracking.poseAngleToTurretAngle(90.0, POSE_OFFSET);
        assertEquals(180.0, result, TOLERANCE);
    }

    @Test
    void poseBehind_clampsToNearest() {
        // Hub at 180° (directly behind) — should map near one limit
        // inputModulus(180 - 360, -180, 180) = inputModulus(-180, -180, 180) = -180
        // result = -180 + 90 = -90 → after clamping = 0
        double raw = TurretTracking.poseAngleToTurretAngle(180.0, POSE_OFFSET);
        // The raw unclamped value should be outside [0, 180], clamped by caller
        assertTrue(raw < 0 || raw > MAX_DEG,
            "Behind-robot bearing should be outside turret range (raw=" + raw + ")");
    }

    // ==================== Clamping edge cases ====================

    @Test
    void exactlyAtZero_accepted() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            5.0, -5.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void exactlyAtMax_accepted() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            170.0, 10.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(180.0, result, TOLERANCE);
    }

    @Test
    void negativeTarget_clampsToZero() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            0.0, -5.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void largeTarget_clampsToMax() {
        double result = TurretTracking.toAbsoluteTurretAngle(
            90.0, 110.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(MAX_DEG, result, TOLERANCE);
    }

    // ==================== computeLeadAngleDeg (isolated) ====================

    @Test
    void stationary_noLeadAngle() {
        // Robot not moving — lead angle should be zero
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 0.0, 3.0, BALL_SPEED);
        assertEquals(0.0, lead, TOLERANCE);
    }

    @Test
    void movingLeft_leadsRight() {
        // Robot heading 0 (facing +X), turret at 90° (forward).
        // Field velocity: vx=0, vy=+2 (moving left in field frame).
        // Cross-track velocity is +2 (leftward relative to aim).
        // Ball drifts left → lead angle should be negative (aim right).
        //
        // Flight time = 3.0 / 15.0 = 0.2s
        // Drift = 2.0 × 0.2 = 0.4m
        // Lead = -atan(0.4 / 3.0) ≈ -7.6°
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 3.0, BALL_SPEED);
        assertEquals(-7.6, lead, 0.2);
        assertTrue(lead < 0, "Moving left should produce negative (rightward) lead");
    }

    @Test
    void movingRight_leadsLeft() {
        // Same setup but moving right (vy = -2)
        // Ball drifts right → lead angle should be positive (aim left)
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, -2.0, 3.0, BALL_SPEED);
        assertEquals(7.6, lead, 0.2);
        assertTrue(lead > 0, "Moving right should produce positive (leftward) lead");
    }

    @Test
    void movingAlongLineOfSight_noLeadAngle() {
        // Robot heading 0, turret at 90° (forward), driving forward (vx=+3, vy=0).
        // All velocity is along the line of sight — no cross-track component.
        // Lead angle should be ~0.
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 3.0, 0.0, 3.0, BALL_SPEED);
        assertEquals(0.0, lead, TOLERANCE);
    }

    @Test
    void leadAngle_independentOfDistance() {
        // Lead angle = atan(v_cross / v_ball). Distance cancels out because
        // both drift and subtended angle scale linearly with distance.
        // So the lead angle should be the same at 2m and 6m.
        double leadClose = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 2.0, BALL_SPEED);
        double leadFar = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 6.0, BALL_SPEED);
        assertEquals(leadClose, leadFar, TOLERANCE,
            "Lead angle should be independent of distance");
    }

    @Test
    void veryCloseDistance_noLeadAngle() {
        // Distance below 0.5m threshold — returns 0 (safety check)
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 0.3, BALL_SPEED);
        assertEquals(0.0, lead, TOLERANCE);
    }

    @Test
    void zeroBallSpeed_noLeadAngle() {
        // Ball speed 0 — returns 0 (division safety)
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 3.0, 0.0);
        assertEquals(0.0, lead, TOLERANCE);
    }

    @Test
    void rotatedRobot_leadStillCorrect() {
        // Robot heading 90° (π/2 rad, facing +Y), turret at 90° (forward).
        // Aim direction in field = π/2 + 0 = π/2 (pointing +Y).
        // Field velocity: vx=+2, vy=0 — robot moving in +X direction.
        // In the aim frame, +X is perpendicular (to the right of aim direction).
        // Cross-track = -2*sin(π/2) + 0*cos(π/2) = -2 (rightward).
        // Ball drifts right → lead should be positive (aim left).
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, Math.PI / 2, 2.0, 0.0, 3.0, BALL_SPEED);
        assertEquals(7.6, lead, 0.2);
        assertTrue(lead > 0, "Should aim left to compensate rightward drift");
    }

    // ==================== Lead angle at turret edge positions ====================
    // These tests verify that lead angle is computed correctly even when the
    // turret is at the edges of its range, not just at center (90°).

    @Test
    void leadAngle_turretAtZero() {
        // Turret at 0° (hard right). Robot heading 0, turret aims at
        // (0 - 90°) = -90° from robot forward = pointing right in robot frame.
        // aimFieldRad = 0 + (-90° in rad) = -π/2.
        // Moving vy=+2 (left in field frame):
        //   cross = -0*sin(-π/2) + 2*cos(-π/2) = 0 + 0 = 0
        // Velocity is along the line of sight (toward the right) — no cross-track.
        double lead = TurretTracking.computeLeadAngleDeg(
            0.0, 0.0, 0.0, 2.0, 3.0, BALL_SPEED);
        assertEquals(0.0, lead, TOLERANCE,
            "Moving left while turret aims right = along line of sight");
    }

    @Test
    void leadAngle_turretAtMax() {
        // Turret at 180° (hard left). aimFieldRad = 0 + (180-90)° = +π/2.
        // Moving vy=+2 (left in field):
        //   cross = -0*sin(π/2) + 2*cos(π/2) = 0 + 0 = 0
        // Velocity is along the line of sight — no cross-track.
        double lead = TurretTracking.computeLeadAngleDeg(
            180.0, 0.0, 0.0, 2.0, 3.0, BALL_SPEED);
        assertEquals(0.0, lead, TOLERANCE,
            "Moving left while turret aims left = along line of sight");
    }

    @Test
    void leadAngle_turretAt45_crossTrack() {
        // Turret at 45°. aimFieldRad = 0 + (45-90)° = -45° = -π/4.
        // Moving vy=+2 (left in field):
        //   cross = -0*sin(-π/4) + 2*cos(-π/4) = 2 * cos(45°) ≈ 1.414
        // Ball drifts left of aim → lead < 0 (aim right)
        double lead = TurretTracking.computeLeadAngleDeg(
            45.0, 0.0, 0.0, 2.0, 3.0, BALL_SPEED);
        assertTrue(lead < 0, "Should aim right to compensate leftward drift");
        // Expected: -atan(1.414 / 15) ≈ -5.4°
        assertEquals(-5.4, lead, 0.3);
    }

    // ==================== Lead angle + target combined (end-to-end) ===========
    // These simulate what autoTrackCommand does: compute a base target from
    // camera/pose, add lead angle, then clamp. This verifies the full flow.

    /** Helper: simulates the combined target computation from autoTrackCommand. */
    private static double computeTrackingTarget(
            double currentAngle, double cameraAngle, boolean cameraHasTarget,
            double poseAngle, boolean poseHasTarget,
            double turretAngle, double robotHeadingRad,
            double vx, double vy, double distance) {
        double baseTarget = TurretTracking.toAbsoluteTurretAngle(
            currentAngle, cameraAngle, cameraHasTarget,
            poseAngle, poseHasTarget, POSE_OFFSET);
        double lead = TurretTracking.computeLeadAngleDeg(
            turretAngle, robotHeadingRad, vx, vy, distance, BALL_SPEED);
        return MathUtil.clamp(baseTarget + lead, 0, MAX_DEG);
    }

    @Test
    void cameraTarget_withLead_offsetsCorrectly() {
        // Camera says hub is 5° right, robot moving left at 2 m/s.
        // Base target = 90 + 5 = 95°
        // Lead ≈ -7.6° (aim right to compensate leftward drift)
        // Combined ≈ 95 - 7.6 = 87.4°
        double result = computeTrackingTarget(
            90.0, 5.0, true, 0.0, false,
            90.0, 0.0, 0.0, 2.0, 3.0);
        assertEquals(87.4, result, 0.3);
    }

    @Test
    void poseTarget_withLead_offsetsCorrectly() {
        // Camera lost, pose says hub ahead (bearing 0° → turret 90°).
        // Robot moving right at 2 m/s.
        // Base target = 90° (from pose)
        // Lead ≈ +7.6° (aim left to compensate rightward drift)
        // Combined ≈ 97.6°
        double result = computeTrackingTarget(
            90.0, 0.0, false, 0.0, true,
            90.0, 0.0, 0.0, -2.0, 3.0);
        assertEquals(97.6, result, 0.3);
    }

    @Test
    void cameraTarget_withLead_clampsAtMax() {
        // Turret currently at 90° (center), camera says hub is 88° right.
        // Base raw = 90 + 88 = 178°.
        // Robot moving left (vy=+2). At turret 90°, cross-track is significant.
        // Lead ≈ -7.6° → combined = 178 + (-7.6) = 170.4° (within range).
        //
        // Without lead, a bigger camera offset (95°) would clamp: 90+95 = 185 → 180.
        // With lead: 90 + 95 + (-7.6) = 177.4 → stays in range, more accurate.
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 2.0, 3.0, BALL_SPEED);
        double withoutLead = MathUtil.clamp(90.0 + 95.0, 0, MAX_DEG);      // 180 (clamped)
        double withLead = MathUtil.clamp(90.0 + 95.0 + lead, 0, MAX_DEG);  // ~177.4
        assertEquals(MAX_DEG, withoutLead, TOLERANCE,
            "Without lead, target should clamp to max");
        assertTrue(withLead < MAX_DEG,
            "Lead angle should pull target back from the clamp boundary (was " + withLead + "°)");
    }

    @Test
    void cameraTarget_withLead_clampsAtMin() {
        // Camera says hub is 3° left of turret at 2°. Base = 2 - 3 = -1 → clamps to 0.
        // Lead is positive (aim left) which makes it worse.
        // Combined: clamp(2 - 3 + lead) → clamp(-1 + lead) → 0 if lead < 1
        double lead = TurretTracking.computeLeadAngleDeg(
            2.0, 0.0, 0.0, -2.0, 3.0, BALL_SPEED);
        double result = MathUtil.clamp(2.0 - 3.0 + lead, 0, MAX_DEG);
        assertEquals(0.0, result, TOLERANCE,
            "Should clamp to 0 when lead pushes target further negative");
    }

    @Test
    void stationaryRobot_leadDoesNotAffectTarget() {
        // Robot stationary — lead = 0, so combined target equals base target.
        double withLead = computeTrackingTarget(
            90.0, 5.0, true, 0.0, false,
            90.0, 0.0, 0.0, 0.0, 3.0);
        double withoutLead = TurretTracking.toAbsoluteTurretAngle(
            90.0, 5.0, true, 0.0, false, POSE_OFFSET);
        assertEquals(withoutLead, withLead, TOLERANCE,
            "Lead angle should be 0 when robot is stationary");
    }

    @Test
    void highSpeed_leadAngleStillReasonable() {
        // Robot at max chassis speed (~4.5 m/s lateral) — lead angle should
        // be significant but not absurd.
        // atan(4.5 / 15) ≈ 16.7°
        double lead = TurretTracking.computeLeadAngleDeg(
            90.0, 0.0, 0.0, 4.5, 3.0, BALL_SPEED);
        assertTrue(Math.abs(lead) < 20.0,
            "Lead angle should stay reasonable even at max speed (was " + lead + "°)");
        assertTrue(Math.abs(lead) > 10.0,
            "Lead angle should be significant at high speed (was " + lead + "°)");
    }
}
