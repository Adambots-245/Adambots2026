// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.simulation;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

import com.adambots.Constants.SimulationConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles FUEL projectile simulation for 3D visualization in AdvantageScope.
 *
 * <p>This class creates realistic ball trajectories using maple-sim's
 * GamePieceProjectile. Trajectories are logged to AdvantageScope for
 * 3D visualization.
 *
 * <p>Physics parameters are based on the 2026 FUEL game piece specifications
 * and typical shooter configurations.
 */
public class FuelProjectile {

    // Hood angle (55 degrees from horizontal)
    private static final Angle HOOD_ANGLE = Degrees.of(SimulationConstants.kHoodAngleDegrees);

    // Launch height from ground
    private static final Distance LAUNCH_HEIGHT = Meters.of(SimulationConstants.kLaunchHeightMeters);
    private static final double LAUNCH_FORWARD_OFFSET_M = 0.25;  // Forward offset from robot center

    // FUEL game piece info for maple-sim
    // Uses dyn4j Circle for the collision shape (2D projection of sphere)
    private static final GamePieceInfo FUEL_INFO = new GamePieceInfo(
        "FUEL",                                                     // type
        new Circle(SimulationConstants.kFuelDiameterMeters / 2.0),  // shape (2D circle)
        Meters.of(SimulationConstants.kFuelDiameterMeters),         // height
        Kilograms.of(SimulationConstants.kFuelMassKg),              // mass
        0.8,   // linearDamping (air resistance approximation)
        0.4,   // angularDamping
        0.6    // coefficientOfRestitution (bounciness)
    );

    // Target position (hub center) - field-relative
    private static final Translation3d HIGH_HUB_CENTER = new Translation3d(
        8.23,   // X position (center of field)
        4.11,   // Y position (center of field)
        SimulationConstants.kHighHubHeightMeters
    );

    // Target tolerance for hit detection (box dimensions)
    private static final Translation3d TARGET_TOLERANCE = new Translation3d(0.5, 0.5, 0.5);

    // NetworkTables publishers for trajectory visualization
    private static StructArrayPublisher<Pose3d> successfulShotPublisher;
    private static StructArrayPublisher<Pose3d> missedShotPublisher;

    static {
        // Initialize NetworkTables publishers for AdvantageScope visualization
        var table = NetworkTableInstance.getDefault().getTable("Shooter");
        successfulShotPublisher = table.getStructArrayTopic("SuccessfulShot", Pose3d.struct).publish();
        missedShotPublisher = table.getStructArrayTopic("MissedShot", Pose3d.struct).publish();
    }

    /**
     * Launches a FUEL projectile from the robot's current position.
     *
     * <p>The projectile trajectory is calculated using physics simulation
     * and logged to AdvantageScope for visualization.
     *
     * @param robotPose The current robot pose on the field
     * @param chassisSpeeds The current chassis speeds (for moving shot compensation)
     * @param exitVelocityMPS The ball exit velocity in meters per second
     */
    public static void launch(Pose2d robotPose, ChassisSpeeds chassisSpeeds, double exitVelocityMPS) {
        // Shooter offset from robot center (forward direction)
        Translation2d shooterOffset = new Translation2d(LAUNCH_FORWARD_OFFSET_M, 0);

        // Convert exit velocity to LinearVelocity
        LinearVelocity launchSpeed = MetersPerSecond.of(exitVelocityMPS);

        // Create the projectile using maple-sim
        GamePieceProjectile projectile = new GamePieceProjectile(
            FUEL_INFO,                               // Game piece info
            robotPose.getTranslation(),              // Robot chassis position
            shooterOffset,                            // Shooter offset from chassis center
            chassisSpeeds,                            // Chassis velocity for moving shots
            robotPose.getRotation(),                  // Shooting direction
            LAUNCH_HEIGHT,                            // Launch height from ground
            launchSpeed,                              // Launch speed
            HOOD_ANGLE                                // Launch angle
        )
        .withTargetPosition(() -> HIGH_HUB_CENTER)
        .withTargetTolerance(TARGET_TOLERANCE)
        .withProjectileTrajectoryDisplayCallBack(
            // Callback for successful shots (hit target zone)
            (List<Pose3d> trajectory) -> {
                successfulShotPublisher.set(trajectory.toArray(new Pose3d[0]));
            },
            // Callback for missed shots
            (List<Pose3d> trajectory) -> {
                missedShotPublisher.set(trajectory.toArray(new Pose3d[0]));
            }
        );

        // Add the projectile to the simulated arena
        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        // Log launch parameters for debugging
        SmartDashboard.putNumber("Shooter/ExitVelocity", exitVelocityMPS);
        SmartDashboard.putNumber("Shooter/LaunchAngleDeg", SimulationConstants.kHoodAngleDegrees);
        SmartDashboard.putNumber("Shooter/RobotX", robotPose.getX());
        SmartDashboard.putNumber("Shooter/RobotY", robotPose.getY());
    }

    /**
     * Launches a FUEL projectile from the robot's current position (stationary shot).
     *
     * @param robotPose The current robot pose on the field
     * @param exitVelocityMPS The ball exit velocity in meters per second
     */
    public static void launch(Pose2d robotPose, double exitVelocityMPS) {
        launch(robotPose, new ChassisSpeeds(), exitVelocityMPS);
    }

    /**
     * Launches a FUEL projectile aimed at a specific target.
     *
     * @param robotPose The current robot pose on the field
     * @param exitVelocityMPS The ball exit velocity in meters per second
     * @param targetPosition The 3D position to aim at
     */
    public static void launchAtTarget(Pose2d robotPose, double exitVelocityMPS, Translation3d targetPosition) {
        // Calculate angle to target
        double dx = targetPosition.getX() - robotPose.getX();
        double dy = targetPosition.getY() - robotPose.getY();
        double angleToTarget = Math.atan2(dy, dx);

        // Create modified robot pose facing the target
        Pose2d aimingPose = new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            new Rotation2d(angleToTarget)
        );

        launch(aimingPose, exitVelocityMPS);
    }

    /**
     * Calculates the exit velocity from flywheel RPS.
     *
     * <p>Formula: v = omega * r * efficiency
     * where omega = 2 * PI * RPS
     *
     * @param flywheelRPS Flywheel rotations per second
     * @return Exit velocity in meters per second
     */
    public static double calculateExitVelocity(double flywheelRPS) {
        double omega = 2 * Math.PI * flywheelRPS;
        return omega * SimulationConstants.kFlywheelRadiusMeters * SimulationConstants.kExitVelocityMultiplier;
    }

    /**
     * Estimates the required RPS to hit a target at a given distance.
     *
     * <p>This is a simplified calculation assuming flat ground and
     * no air resistance. For better accuracy, use the interpolating
     * table in ShooterSubsystem.
     *
     * @param distanceMeters Horizontal distance to target
     * @return Estimated flywheel RPS needed
     */
    public static double estimateRequiredRPS(double distanceMeters) {
        // Use projectile motion equations to estimate required velocity
        double g = 9.81;
        double theta = Math.toRadians(SimulationConstants.kHoodAngleDegrees);
        double heightDiff = SimulationConstants.kHighHubHeightMeters - SimulationConstants.kLaunchHeightMeters;

        // Formula accounting for height difference:
        // v^2 = (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - h))
        double d = distanceMeters;
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        double numerator = g * d * d;
        double denominator = 2 * cosTheta * cosTheta * (d * tanTheta - heightDiff);

        if (denominator <= 0) {
            // Target is too high/close to hit with this hood angle
            return 100.0;  // Return max RPS
        }

        double requiredVelocity = Math.sqrt(numerator / denominator);

        // Convert velocity to RPS
        double omega = requiredVelocity / (SimulationConstants.kFlywheelRadiusMeters * SimulationConstants.kExitVelocityMultiplier);
        return omega / (2 * Math.PI);
    }
}
