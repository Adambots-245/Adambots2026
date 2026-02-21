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

    // ==================== REAR CAMERA ====================
    private PhotonCamera rearCamera;
    private Transform3d robotToRearCamera;

    // ==================== SIMULATION ====================
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim rearCameraSim;

    // ==================== STATE (FRONT) ====================
    private double lastDistanceToTarget = 0.0;
    private double lastTargetYawDegrees = 0.0;
    private boolean hasTarget = false;
    private int targetId = -1;

    // ==================== STATE (REAR) ====================
    private double rearLastDistanceToTarget = 0.0;
    private double rearLastTargetYawDegrees = 0.0;
    private boolean rearHasTarget = false;
    private int rearTargetId = -1;

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
     * Initialize a rear-facing camera and add it to the existing VisionSystemSim.
     * Call this after construction (and only in simulation mode).
     *
     * @param name Camera name (must match PhotonVision config)
     * @param transform Camera position/rotation relative to robot center
     */
    public void initRearCamera(String name, Transform3d transform) {
        rearCamera = new PhotonCamera(name);
        robotToRearCamera = transform;

        if (RobotBase.isSimulation() && visionSim != null) {
            SimCameraProperties rearProp = new SimCameraProperties();
            rearProp.setCalibration(
                SimulationConstants.kCameraResolutionWidth,
                SimulationConstants.kCameraResolutionHeight,
                Rotation2d.fromDegrees(SimulationConstants.kCameraFOVDegrees)
            );
            rearProp.setCalibError(0.25, 0.08);
            rearProp.setFPS(30);
            rearProp.setAvgLatencyMs(35);
            rearProp.setLatencyStdDevMs(5);

            rearCameraSim = new PhotonCameraSim(rearCamera, rearProp);
            rearCameraSim.enableDrawWireframe(true);
            visionSim.addCamera(rearCameraSim, robotToRearCamera);
        }
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

                // Get yaw angle to target (degrees, positive = target is to the left)
                lastTargetYawDegrees = bestTarget.getYaw();
            } else {
                targetId = -1;
                // Keep the last known distance and yaw (don't reset)
            }
        }

        // Process rear camera results
        if (rearCamera != null) {
            var rearResults = rearCamera.getAllUnreadResults();
            if (!rearResults.isEmpty()) {
                PhotonPipelineResult rearResult = rearResults.get(rearResults.size() - 1);
                rearHasTarget = rearResult.hasTargets();

                if (rearHasTarget) {
                    PhotonTrackedTarget bestRear = rearResult.getBestTarget();
                    rearTargetId = bestRear.getFiducialId();

                    var rearTransform = bestRear.getBestCameraToTarget();
                    rearLastDistanceToTarget = Math.sqrt(
                        rearTransform.getX() * rearTransform.getX() +
                        rearTransform.getY() * rearTransform.getY()
                    );
                    rearLastTargetYawDegrees = bestRear.getYaw();
                } else {
                    rearTargetId = -1;
                }
            }
        }

        // Publish live telemetry for simulation testing
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/Distance", lastDistanceToTarget);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/TargetYaw", lastTargetYawDegrees);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/TagId", targetId);

        // Rear camera telemetry
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Vision/RearHasTarget", rearHasTarget);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/RearDistance", rearLastDistanceToTarget);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/RearTargetYaw", rearLastTargetYawDegrees);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Vision/RearTagId", rearTargetId);
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
     * Gets the yaw angle to the currently tracked target.
     * Positive means the target is to the left of the camera center.
     *
     * @return Yaw angle in degrees, or the last known yaw if no target
     */
    public double getTargetYawDegrees() {
        return lastTargetYawDegrees;
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

    // ==================== REAR CAMERA GETTERS ====================

    /**
     * Checks if the rear camera currently sees a valid target.
     *
     * @return true if a target is visible on the rear camera
     */
    public boolean rearHasTarget() {
        return rearHasTarget;
    }

    /**
     * Gets the distance to the rear camera's currently tracked target.
     *
     * @return Distance in meters, or last known distance if no target
     */
    public double getRearDistanceToTarget() {
        return rearLastDistanceToTarget;
    }

    /**
     * Gets the yaw angle to the rear camera's currently tracked target.
     * Positive means target is to the left of the camera center.
     *
     * @return Yaw angle in degrees
     */
    public double getRearTargetYawDegrees() {
        return rearLastTargetYawDegrees;
    }

    /**
     * Gets the AprilTag ID of the rear camera's currently tracked target.
     *
     * @return The fiducial ID, or -1 if no target
     */
    public int getRearTargetId() {
        return rearTargetId;
    }

    /**
     * Gets the best target from the rear camera that matches one of the allowed tag IDs.
     * Useful for filtering to only the correct tower's tags.
     *
     * @param allowedIds Array of allowed AprilTag fiducial IDs
     * @return The best matching PhotonTrackedTarget, or null if none match
     */
    public PhotonTrackedTarget getRearTargetFiltered(int[] allowedIds) {
        if (rearCamera == null) return null;

        var results = rearCamera.getAllUnreadResults();
        if (results.isEmpty()) return null;

        PhotonPipelineResult result = results.get(results.size() - 1);
        if (!result.hasTargets()) return null;

        for (PhotonTrackedTarget target : result.getTargets()) {
            int id = target.getFiducialId();
            for (int allowed : allowedIds) {
                if (id == allowed) {
                    return target;
                }
            }
        }
        return null;
    }
}
