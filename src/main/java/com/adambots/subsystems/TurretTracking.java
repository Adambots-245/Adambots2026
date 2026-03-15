package com.adambots.subsystems;

import com.adambots.Constants.TurretConstants;
import edu.wpi.first.math.MathUtil;

/**
 * Pure-math helper for turret angle calculations. Extracted for unit testing
 * without requiring hardware (BaseMotor).
 */
final class TurretTracking {
    private TurretTracking() {}

    /**
     * Converts a robot-relative pose bearing to an unclamped turret angle.
     * Uses centered inputModulus so that bearings outside the turret's arc
     * map to the nearest limit after clamping, rather than wrapping to the far limit.
     */
    static double poseAngleToTurretAngle(double poseAngle, double poseOffsetDegrees) {
        double turretCenterBearing = poseOffsetDegrees + TurretConstants.kTurretMaxDegrees / 2.0;
        return MathUtil.inputModulus(poseAngle - turretCenterBearing, -180, 180)
               + TurretConstants.kTurretMaxDegrees / 2.0;
    }

    /**
     * Selects the best turret target angle based on available sensor inputs.
     * Camera angle is a turret-relative offset; pose angle is a robot-relative bearing.
     * Returns the target clamped to [0, kTurretMaxDegrees], or currentTurretAngle if nothing available.
     */
    static double toAbsoluteTurretAngle(double currentTurretAngle,
                                         double cameraAngle, boolean cameraHasTarget,
                                         double poseAngle, boolean poseHasTarget,
                                         double poseOffsetDegrees) {
        double target;
        if (cameraHasTarget) {
            target = currentTurretAngle + cameraAngle;
        } else if (poseHasTarget) {
            target = poseAngleToTurretAngle(poseAngle, poseOffsetDegrees);
        } else {
            return currentTurretAngle; // hold position
        }
        return MathUtil.clamp(target, 0, TurretConstants.kTurretMaxDegrees);
    }

    /**
     * Computes lead angle compensation for shooting while the robot is moving.
     *
     * <h3>Why we need this:</h3>
     * When the robot drives sideways, the ball inherits that sideways velocity.
     * Even if the turret points directly at the hub, the ball will drift off-target
     * during its flight. We aim the turret slightly "into" the robot's motion to cancel
     * the drift.
     *
     * <h3>How the math works:</h3>
     * <pre>
     *   1. Convert robot's field velocity into the turret's aim frame
     *   2. Find the "cross-track" component — the part of the velocity
     *      that is perpendicular to the line-of-sight to the hub
     *   3. Estimate flight time = distance / ballSpeed
     *   4. Drift = crossTrackVelocity × flightTime (meters the ball will miss by)
     *   5. Lead angle = atan(drift / distance) — aim this many degrees opposite
     *
     *   Example: robot drives left at 2 m/s, hub is 3m away, ball speed 15 m/s
     *     → flight time = 3/15 = 0.2s
     *     → drift = 2 × 0.2 = 0.4m
     *     → lead angle = atan(0.4/3) ≈ 7.6°
     *     → aim 7.6° to the RIGHT to compensate
     * </pre>
     *
     * @param turretAngleDeg     current turret aim angle (0-180° turret frame)
     * @param robotHeadingRad    robot heading in field frame (radians, CCW positive)
     * @param fieldVxMps         robot X velocity in field frame (m/s, +X = toward red alliance wall)
     * @param fieldVyMps         robot Y velocity in field frame (m/s, +Y = toward left wall)
     * @param distanceM          distance to hub (meters)
     * @param ballSpeedMps       estimated ball exit speed (m/s)
     * @return lead angle offset in degrees (add to turret target to compensate)
     */
    static double computeLeadAngleDeg(double turretAngleDeg, double robotHeadingRad,
                                       double fieldVxMps, double fieldVyMps,
                                       double distanceM, double ballSpeedMps) {
        // Don't compute lead if distance is invalid or too small
        if (distanceM < 0.5 || ballSpeedMps <= 0) {
            return 0.0;
        }

        // Step 1: Find the turret's aim direction in the field frame.
        // Turret 170° = robot forward (kTurretForwardDegrees = 170°).
        // So the turret-to-robot offset is (turretAngle - 90°).
        // Adding the robot heading gives us the field-frame aim direction.
        double aimFieldRad = robotHeadingRad
            + Math.toRadians(turretAngleDeg - TurretConstants.kTurretForwardDegrees);

        // Step 2: Compute the cross-track velocity.
        // This is the component of robot velocity PERPENDICULAR to the aim direction.
        // Positive = robot moving left relative to aim direction.
        //
        // The perpendicular (left-pointing) unit vector is (-sin(aim), cos(aim)).
        // Dot product with velocity gives the cross-track component.
        double crossTrackVel = -fieldVxMps * Math.sin(aimFieldRad)
                              + fieldVyMps * Math.cos(aimFieldRad);

        // Step 3: Estimate how long the ball is in the air.
        double flightTimeSec = distanceM / ballSpeedMps;

        // Step 4: How far the ball will drift during flight.
        double driftM = crossTrackVel * flightTimeSec;

        // Step 5: Convert drift to an angle offset.
        // Negative sign: if robot moves LEFT (positive crossTrack), the ball drifts
        // LEFT, so we need to aim RIGHT (negative offset) to compensate.
        return -Math.toDegrees(Math.atan2(driftM, distanceM));
    }
}
