// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.adambots.Constants.SimulationConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for simulating PhotonVision AprilTag detection.
 *
 * <p>This subsystem provides:
 * <ul>
 *   <li>PhotonCamera for real robot operation</li>
 *   <li>VisionSystemSim with PhotonCameraSim for simulation</li>
 *   <li>Distance calculation to visible AprilTag targets</li>
 * </ul>
 *
 * <p>In simulation, call simulationPeriodic() with the robot pose to update
 * the simulated camera's view of AprilTags on the field.
 */
@Logged
public class VisionSimSubsystem extends SubsystemBase {

    // ==================== HARDWARE ====================
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;

    // ==================== SIMULATION ====================
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    // ==================== STATE ====================
    private double lastDistanceToTarget = 0.0;
    private boolean hasTarget = false;
    private int targetId = -1;

    /**
     * Creates a new VisionSimSubsystem.
     *
     * @param cameraName The name of the PhotonVision camera (as configured in PhotonVision UI)
     */
    public VisionSimSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);

        // Configure camera position relative to robot center
        robotToCamera = new Transform3d(
            new Translation3d(
                SimulationConstants.kCameraXMeters,
                SimulationConstants.kCameraYMeters,
                SimulationConstants.kCameraZMeters
            ),
            new Rotation3d(
                0,  // Roll
                Math.toRadians(SimulationConstants.kCameraPitchDegrees),
                Math.toRadians(SimulationConstants.kCameraYawDegrees)
            )
        );

        // Initialize simulation components if running in simulation
        if (RobotBase.isSimulation()) {
            simulationInit();
        }
    }

    /**
     * Initialize simulation components.
     * Called automatically in simulation mode.
     */
    private void simulationInit() {
        // Create the vision system simulation
        visionSim = new VisionSystemSim("main");

        // Load the AprilTag field layout
        // Try to load the 2026 field, fall back to 2024 Crescendo if not available
        AprilTagFieldLayout fieldLayout = null;
        try {
            // First try the welded layout path for 2026
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            System.err.println("Failed to load default field layout: " + e.getMessage());
        }

        if (fieldLayout != null) {
            visionSim.addAprilTags(fieldLayout);
        } else {
            System.err.println("Warning: No AprilTag field layout loaded for vision simulation");
        }

        // Configure simulated camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
            SimulationConstants.kCameraResolutionWidth,
            SimulationConstants.kCameraResolutionHeight,
            Rotation2d.fromDegrees(SimulationConstants.kCameraFOVDegrees)
        );
        cameraProp.setCalibError(0.25, 0.08);  // Average and std dev of calibration error (pixels)
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // Create the camera simulation and add it to the vision system
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);  // Draw wireframe in AdvantageScope
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    /**
     * Update the vision simulation with the current robot pose.
     * Must be called from Robot.simulationPeriodic().
     *
     * @param robotPose The current robot pose on the field
     */
    public void simulationPeriodic(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    @Override
    public void periodic() {
        // Process camera results using the new API (getAllUnreadResults)
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Get the most recent result
            PhotonPipelineResult result = results.get(results.size() - 1);
            hasTarget = result.hasTargets();

            if (hasTarget) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                targetId = bestTarget.getFiducialId();

                // Calculate distance to target using the target's transform
                // The transform gives us the 3D position of the target relative to the camera
                var transform = bestTarget.getBestCameraToTarget();
                lastDistanceToTarget = Math.sqrt(
                    transform.getX() * transform.getX() +
                    transform.getY() * transform.getY()
                );
            } else {
                targetId = -1;
                // Keep the last known distance (don't reset to 0)
            }
        }
    }

    // ==================== PUBLIC GETTERS ====================

    /**
     * Gets the distance to the currently tracked target.
     *
     * @return Distance in meters to the target, or the last known distance if no target
     */
    public double getDistanceToTarget() {
        return lastDistanceToTarget;
    }

    /**
     * Checks if the camera currently sees a valid target.
     *
     * @return true if a target is visible
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Gets the AprilTag ID of the currently tracked target.
     *
     * @return The fiducial ID, or -1 if no target
     */
    public int getTargetId() {
        return targetId;
    }

    /**
     * Gets the PhotonCamera for direct access if needed.
     *
     * @return The PhotonCamera instance
     */
    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * Gets the camera-to-robot transform.
     *
     * @return The Transform3d from robot center to camera
     */
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }
}
