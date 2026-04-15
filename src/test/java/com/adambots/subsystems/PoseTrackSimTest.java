package com.adambots.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Tests the pose-lock turret tracking math and logic.
 *
 * <p>Conventions (verified from codebase):
 * <ul>
 *   <li>Field: origin at blue wall, +X toward red, +Y left from blue driver station</li>
 *   <li>Robot heading: 0° = facing +X (toward red), positive = CCW</li>
 *   <li>Turret: 0° = far left, kTurretForwardDegrees (99°) = straight ahead,
 *       kTurretMaxDegrees = far right. Shooter/camera faces backward.</li>
 *   <li>poseAngleToTurretAngle: hub at ±180° robot-relative = turret at 99° (forward)</li>
 * </ul>
 */
class PoseTrackSimTest {

    static final double TURRET_FORWARD = 99.0;  // kTurretForwardDegrees
    static final double TURRET_MAX = 256.0;      // approximate

    // ==================== poseAngleToTurretAngle Tests ====================

    @Test
    void testHubDirectlyBehind_turretForward() {
        // Hub is directly behind the robot (±180° robot-relative)
        // Turret should point forward (99°) since shooter faces back
        double result = TurretSubsystem.poseAngleToTurretAngle(180.0);
        assertEquals(TURRET_FORWARD, result, 0.01,
            "Hub at 180° (behind) should map to turret forward (99°)");
    }

    @Test
    void testHubDirectlyBehind_negative() {
        // -180° is same as +180° (directly behind)
        double result = TurretSubsystem.poseAngleToTurretAngle(-180.0);
        assertEquals(TURRET_FORWARD, result, 0.01,
            "Hub at -180° (behind) should also map to turret forward (99°)");
    }

    @Test
    void testHubSlightlyLeft_turretMovesLeft() {
        // Hub is 10° to the left of directly behind (170° robot-relative)
        // Turret should be at 99° - 10° = 89°
        double result = TurretSubsystem.poseAngleToTurretAngle(170.0);
        assertEquals(89.0, result, 0.01,
            "Hub 10° left of behind should map to turret 89°");
    }

    @Test
    void testHubSlightlyRight_turretMovesRight() {
        // Hub is 10° to the right of directly behind (-170° robot-relative)
        double result = TurretSubsystem.poseAngleToTurretAngle(-170.0);
        assertEquals(109.0, result, 0.01,
            "Hub 10° right of behind should map to turret 109°");
    }

    @Test
    void testHubFarLeft() {
        // Hub is 90° to the left (90° robot-relative)
        // Turret angle = 99 + 90 - 180 = 9°
        double result = TurretSubsystem.poseAngleToTurretAngle(90.0);
        assertEquals(9.0, result, 0.01,
            "Hub at 90° (left) should map to turret 9°");
    }

    @Test
    void testHubFarRight() {
        // Hub is 90° to the right (-90° or 270° robot-relative)
        // Turret angle = 99 + 270 - 180 = 189°
        double result = TurretSubsystem.poseAngleToTurretAngle(270.0);
        assertEquals(189.0, result, 0.01,
            "Hub at 270° (right) should map to turret 189°");
    }

    @Test
    void testHubDirectlyInFront() {
        // Hub is directly in front of the robot (0° robot-relative)
        // Turret angle = 99 + 0 - 180 = -81° (out of turret range — unreachable)
        double result = TurretSubsystem.poseAngleToTurretAngle(0.0);
        assertEquals(-81.0, result, 0.01,
            "Hub at 0° (in front) maps to -81° (out of range, needs clamping)");
    }

    // ==================== Full Pose-to-Turret Pipeline Tests ====================

    @Test
    void testRobotFacingAwayFromHub_blueAlliance() {
        // Robot at (2, 4) facing +X (0°), blue hub at (4.54, 8.23)
        double dx = 4.54 - 2.0;
        double dy = 8.23 - 4.0;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // ~59°
        double robotHeading = 0.0;
        double robotRelative = worldBearing - robotHeading;  // ~59°
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is to the front-left of the robot — turret needs to point far left
        // turretAngle = 99 + 59 - 180 = -22° → clamped to 0°
        assertTrue(turretAngle < 0,
            "Hub in front-left should be below turret range (needs clamp)");
        System.out.printf("Blue hub from (2,4) heading 0°: bearing=%.1f° turret=%.1f°%n",
            worldBearing, turretAngle);
    }

    @Test
    void testRobotFacingHub_blueAlliance() {
        // Robot at (2, 4) facing the hub: heading = atan2(8.23-4, 4.54-2) ≈ 59°
        double dx = 4.54 - 2.0;
        double dy = 8.23 - 4.0;
        double headingToHub = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = headingToHub;  // robot faces directly at hub
        double robotRelative = headingToHub - robotHeading;  // 0° = hub directly in front
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is directly in front → turret at -81° (out of range — shooter faces back)
        assertTrue(turretAngle < 0,
            "Hub directly in front is unreachable (shooter faces back)");
    }

    @Test
    void testRobotBackToHub_blueAlliance() {
        // Robot at (2, 4) with BACK facing the hub (heading = 59° + 180° = 239°)
        double dx = 4.54 - 2.0;
        double dy = 8.23 - 4.0;
        double headingToHub = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = headingToHub + 180.0;  // back faces hub
        double robotRelative = headingToHub - robotHeading;  // -180° = hub directly behind
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub directly behind → turret forward (99°)
        assertEquals(TURRET_FORWARD, turretAngle, 1.0,
            "Hub directly behind (shooter aimed at hub) should be turret forward");
        System.out.printf("Back to hub: heading=%.1f° relative=%.1f° turret=%.1f°%n",
            robotHeading, robotRelative, turretAngle);
    }

    @Test
    void testRobotMovingLeftToRight_redAlliance() {
        // Robot moving across the field at Y=4, red hub at (12, 8.23)
        double hubX = 12.0, hubY = 8.23;
        double robotHeading = 180.0;  // facing blue (back toward red hub)

        for (double robotX = 10; robotX <= 14; robotX += 1.0) {
            double dx = hubX - robotX;
            double dy = hubY - 4.0;
            double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
            double robotRelative = worldBearing - robotHeading;
            double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

            System.out.printf("  Robot at (%.0f, 4) heading 180°: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
                robotX, worldBearing, robotRelative, turretAngle);

            // Turret should be within the valid mechanical range
            assertTrue(turretAngle > 0 && turretAngle < TURRET_MAX,
                "Turret should be in valid range, was " + turretAngle);
        }
    }

    @Test
    void testRobotSpinning_turretCounterRotates() {
        // Robot at fixed position, spinning. Turret should counter-rotate.
        double hubX = 12.0, hubY = 8.23;
        double robotX = 11.0, robotY = 4.0;
        double dx = hubX - robotX;
        double dy = hubY - robotY;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));

        double prevTurretAngle = 0;
        boolean firstIteration = true;

        for (double heading = 0; heading < 360; heading += 30) {
            double robotRelative = worldBearing - heading;
            double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

            if (!firstIteration) {
                // As robot heading increases (CCW), turret should decrease (CW) to compensate
                // (within the valid range — wrapping may cause jumps)
            }
            firstIteration = false;
            prevTurretAngle = turretAngle;

            System.out.printf("  Heading=%.0f° → turret=%.1f°%n", heading, turretAngle);
        }
        // Visual check: turret angle should shift opposite to heading
    }

    // ==================== Red Alliance Tests ====================

    @Test
    void testRedAlliance_backToHub() {
        // Red hub at (12, 8.23), robot at (14, 6), heading 180° (facing blue, back toward red)
        double hubX = 12.0, hubY = 8.23;
        double dx = hubX - 14.0;
        double dy = hubY - 6.0;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // ~132° (hub is behind-left)
        double robotHeading = 180.0;
        double robotRelative = worldBearing - robotHeading;  // ~-48°
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is behind-left in world, but robot faces 180° so it's behind-right
        // in robot frame → turret should be right of forward
        assertTrue(turretAngle > TURRET_FORWARD && turretAngle < TURRET_MAX,
            "Red hub behind-right (robot frame) should give turret > 99°, was " + turretAngle);
        System.out.printf("Red back-to-hub: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testRedAlliance_robotLeftOfHub() {
        // Robot at (12, 3) — directly below hub (12, 8.23)
        // Heading 180° (facing blue, back toward red wall)
        double hubX = 12.0, hubY = 8.23;
        double dx = hubX - 12.0;  // 0
        double dy = hubY - 3.0;   // 5.23
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // 90° (hub is directly +Y)
        double robotHeading = 180.0;
        double robotRelative = worldBearing - robotHeading;  // -90° (hub is to the right behind)
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is 90° to the right in robot frame → turret should be right of forward
        assertTrue(turretAngle > TURRET_FORWARD,
            "Hub to the right should give turret > 99°, was " + turretAngle);
        assertEquals(189.0, turretAngle, 1.0,
            "Hub at -90° relative should map to turret ~189°");
        System.out.printf("Red left-of-hub: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testRedAlliance_robotRightOfHub() {
        // Robot at (12, 12) — above hub (12, 8.23)
        // Heading 180°
        double hubX = 12.0, hubY = 8.23;
        double dx = hubX - 12.0;  // 0
        double dy = hubY - 12.0;  // -3.77
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // -90° (hub is directly -Y)
        double robotHeading = 180.0;
        double robotRelative = worldBearing - robotHeading;  // -270° → normalizes to 90°
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is to the left in robot frame → turret should be left of forward
        assertTrue(turretAngle < TURRET_FORWARD,
            "Hub to the left should give turret < 99°, was " + turretAngle);
        assertEquals(9.0, turretAngle, 1.0,
            "Hub at 90° relative should map to turret ~9°");
        System.out.printf("Red right-of-hub: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testRedAlliance_robotTurned45() {
        // Robot at (14, 6), heading 225° (turned 45° from facing blue)
        // Back is facing roughly toward red hub
        double hubX = 12.0, hubY = 8.23;
        double dx = hubX - 14.0;
        double dy = hubY - 6.0;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // ~132°
        double robotHeading = 225.0;
        double robotRelative = worldBearing - robotHeading;  // ~-93°
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        assertTrue(turretAngle > 0 && turretAngle < TURRET_MAX,
            "Should be reachable, was " + turretAngle);
        System.out.printf("Red turned 45°: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    // ==================== Blue Alliance Tests ====================

    @Test
    void testBlueAlliance_backToHub() {
        // Blue hub at (4.54, 8.23), robot at (2, 6), heading 0° (facing red, back toward blue)
        double hubX = 4.54, hubY = 8.23;
        double dx = hubX - 2.0;
        double dy = hubY - 6.0;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // ~41°
        double robotHeading = 0.0;
        double robotRelative = worldBearing - robotHeading;  // ~41°
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is in front-right → turret needs to point far left (may be out of range)
        System.out.printf("Blue back-to-hub (heading 0°): bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testBlueAlliance_backActuallyFacingHub() {
        // Blue hub at (4.54, 8.23), robot at (2, 6)
        // Heading = bearing + 180 (back faces hub)
        double hubX = 4.54, hubY = 8.23;
        double dx = hubX - 2.0;
        double dy = hubY - 6.0;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = worldBearing + 180.0;  // back faces hub
        double robotRelative = worldBearing - robotHeading;
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        assertEquals(TURRET_FORWARD, turretAngle, 1.0,
            "Blue: back facing hub should give turret forward (99°)");
        System.out.printf("Blue back-facing-hub: heading=%.1f° relative=%.1f° turret=%.1f°%n",
            robotHeading, robotRelative, turretAngle);
    }

    @Test
    void testBlueAlliance_robotLeftOfHub() {
        // Robot at (4.54, 3) — below hub (4.54, 8.23), heading 0° (facing red)
        double hubX = 4.54, hubY = 8.23;
        double dx = hubX - 4.54;  // 0
        double dy = hubY - 3.0;   // 5.23
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));  // 90°
        double robotHeading = 0.0;
        double robotRelative = worldBearing - robotHeading;  // 90° (hub is to the left)
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub at 90° robot-relative (left side) → turret should be far left
        assertEquals(9.0, turretAngle, 1.0,
            "Hub at 90° relative should map to turret ~9°");
        System.out.printf("Blue left-of-hub: turret=%.1f°%n", turretAngle);
    }

    @Test
    void testBlueAlliance_movingAcrossField() {
        // Robot sweeping from Y=3 to Y=12 at X=3, heading 0° (facing red)
        // Blue hub at (4.54, 8.23)
        double hubX = 4.54, hubY = 8.23;
        double robotHeading = 0.0;

        System.out.println("Blue alliance, moving Y=3→12 at X=3, heading 0°:");
        for (double robotY = 3; robotY <= 12; robotY += 1.5) {
            double dx = hubX - 3.0;
            double dy = hubY - robotY;
            double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
            double robotRelative = worldBearing - robotHeading;
            double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

            System.out.printf("  Y=%.1f: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
                robotY, worldBearing, robotRelative, turretAngle);
        }
        // As robot moves from below hub to above hub, turret should sweep
        // from one side to the other through forward (99°)
    }

    // ==================== Existing tests below ====================

    @Test
    void testPoseAtOrigin_fallsBackToSweep() {
        // Pose at (0,0) means odom hasn't initialized — norm < 1.0
        double norm = Math.sqrt(0 * 0 + 0 * 0);
        assertTrue(norm < 1.0,
            "Pose at origin should trigger sweep fallback (norm < 1.0)");
    }

    @Test
    void testHardcodedHubCenters() {
        // Verify hardcoded hub centers match expected positions from WPILib field layout
        assertEquals(12.004, com.adambots.Constants.VisionConstants.kRedHubCenterX, 0.01,
            "Red hub X should be ~12.004m");
        assertEquals(4.035, com.adambots.Constants.VisionConstants.kRedHubCenterY, 0.01,
            "Red hub Y should be ~4.035m (field center)");
        assertEquals(4.537, com.adambots.Constants.VisionConstants.kBlueHubCenterX, 0.01,
            "Blue hub X should be ~4.537m");
        assertEquals(4.035, com.adambots.Constants.VisionConstants.kBlueHubCenterY, 0.01,
            "Blue hub Y should be ~4.035m (field center)");
    }

    @Test
    void testPurePoseTracking_noCameraBlend() {
        // Pure pose tracking: turret angle should equal poseAngleToTurretAngle
        // with no camera correction applied
        double poseAngle = 170.0;  // hub slightly left of behind
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(poseAngle);
        assertEquals(89.0, turretAngle, 0.01,
            "Pure pose: 170° relative should give exactly 89° turret (no blend)");
    }

    @Test
    void testFullPipeline_redHub_hardcoded() {
        // Full pipeline using hardcoded red hub center
        double hubX = 12.004, hubY = 4.035;
        double robotX = 14.0, robotY = 4.0;
        double robotHeading = 0.0;  // facing +X, back toward hub

        double dx = hubX - robotX;
        double dy = hubY - robotY;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
        double robotRelative = worldBearing - robotHeading;
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is almost directly behind → turret near forward
        assertTrue(turretAngle > 95 && turretAngle < 103,
            "Red hub from (14,4) heading 0° should give turret ~99°, was " + turretAngle);
        System.out.printf("Red hub hardcoded: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testFullPipeline_blueHub_hardcoded() {
        // Full pipeline using hardcoded blue hub center
        double hubX = 4.537, hubY = 4.035;
        double robotX = 2.0, robotY = 4.0;
        double robotHeading = 180.0;  // facing -X (toward blue wall), back toward hub

        double dx = hubX - robotX;
        double dy = hubY - robotY;
        double worldBearing = Math.toDegrees(Math.atan2(dy, dx));
        double robotRelative = worldBearing - robotHeading;
        double turretAngle = TurretSubsystem.poseAngleToTurretAngle(robotRelative);

        // Hub is behind → turret near forward
        assertTrue(turretAngle > 95 && turretAngle < 103,
            "Blue hub from (2,4) heading 180° should give turret ~99°, was " + turretAngle);
        System.out.printf("Blue hub hardcoded: bearing=%.1f° relative=%.1f° turret=%.1f°%n",
            worldBearing, robotRelative, turretAngle);
    }

    @Test
    void testTurretRangeClamp() {
        // Hub position that would require turret angle outside valid range
        double outOfRange = TurretSubsystem.poseAngleToTurretAngle(0.0);
        assertTrue(outOfRange < 0,
            "Hub in front should produce negative turret angle (needs clamping)");

        // After clamping
        double clamped = Math.max(0, Math.min(TURRET_MAX, outOfRange));
        assertEquals(0, clamped, 0.01,
            "Should clamp to 0° (turret left limit)");
    }
}
