// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.HangConstants;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.subsystems.VisionSimSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for hang alignment using a rear-facing PhotonVision camera.
 *
 * <p>Sequence:
 * <ol>
 *   <li>Drive to approach pose (~1.5m from wall, back facing tower)</li>
 *   <li>3-DOF PID vision alignment: rotation + lateral + distance</li>
 * </ol>
 */
public final class HangAlignCommands {

    private HangAlignCommands() {}

    /**
     * Creates the full hang alignment command sequence.
     *
     * @param swerve The swerve drive subsystem
     * @param visionSim The vision simulation subsystem (with rear camera initialized)
     * @param isRedAlliance true if on red alliance
     * @param useUpperTower true for upper tower, false for lower
     * @return Command that drives to approach zone then vision-aligns to tower
     */
    public static Command hangAlignCommand(
            SwerveSubsystem swerve, VisionSimSubsystem visionSim,
            boolean isRedAlliance, boolean useUpperTower) {

        Pose2d approachPose = HangConstants.getApproachPose(isRedAlliance, useUpperTower);
        int[] towerTags = HangConstants.getTowerTags(isRedAlliance, useUpperTower);
        double targetHeadingRad = HangConstants.getTargetHeadingRad(isRedAlliance);

        return Commands.sequence(
            // Phase 1: Status update
            Commands.runOnce(() -> SmartDashboard.putString("Hang/Status", "DRIVING TO APPROACH")),

            // Phase 2: Drive to approach pose
            swerve.driveToPoseCommand(approachPose).withTimeout(8),

            // Phase 3: Vision-aligned final positioning
            Commands.runOnce(() -> SmartDashboard.putString("Hang/Status", "VISION ALIGNING")),

            visionAlignCommand(swerve, visionSim, towerTags, targetHeadingRad)
                .withTimeout(10)

        ).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Hang/Status", "ALIGNED");
        }).withName("HangAlign");
    }

    /**
     * Creates the 3-DOF PID vision alignment command.
     * Uses rear camera to simultaneously correct rotation, lateral offset, and distance.
     */
    private static Command visionAlignCommand(
            SwerveSubsystem swerve, VisionSimSubsystem visionSim,
            int[] towerTags, double targetHeadingRad) {

        return Commands.defer(() -> {
            // Read tunable gains from dashboard (with defaults from constants)
            PIDController rotPID = new PIDController(
                SmartDashboard.getNumber("Hang/RotP", HangConstants.kRotP),
                HangConstants.kRotI,
                SmartDashboard.getNumber("Hang/RotD", HangConstants.kRotD));
            rotPID.enableContinuousInput(-Math.PI, Math.PI);
            rotPID.setTolerance(Math.toRadians(HangConstants.kRotToleranceDeg));

            PIDController latPID = new PIDController(
                SmartDashboard.getNumber("Hang/LatP", HangConstants.kLatP),
                HangConstants.kLatI,
                SmartDashboard.getNumber("Hang/LatD", HangConstants.kLatD));
            latPID.setTolerance(HangConstants.kLatToleranceMeters);

            PIDController distPID = new PIDController(
                SmartDashboard.getNumber("Hang/DistP", HangConstants.kDistP),
                HangConstants.kDistI,
                SmartDashboard.getNumber("Hang/DistD", HangConstants.kDistD));
            distPID.setTolerance(HangConstants.kDistToleranceMeters);

            return Commands.run(() -> {
                // Live-update gains
                rotPID.setP(SmartDashboard.getNumber("Hang/RotP", HangConstants.kRotP));
                rotPID.setD(SmartDashboard.getNumber("Hang/RotD", HangConstants.kRotD));
                latPID.setP(SmartDashboard.getNumber("Hang/LatP", HangConstants.kLatP));
                latPID.setD(SmartDashboard.getNumber("Hang/LatD", HangConstants.kLatD));
                distPID.setP(SmartDashboard.getNumber("Hang/DistP", HangConstants.kDistP));
                distPID.setD(SmartDashboard.getNumber("Hang/DistD", HangConstants.kDistD));

                boolean hasTarget = visionSim.rearHasTarget();
                SmartDashboard.putBoolean("Hang/RearHasTarget", hasTarget);

                double omega = 0;
                double vx = 0; // forward/back (robot-relative)
                double vy = 0; // left/right (robot-relative)

                // Rotation: align heading so back faces wall
                double currentHeadingRad = swerve.getPose().getRotation().getRadians();
                omega = rotPID.calculate(currentHeadingRad, targetHeadingRad);
                omega = MathUtil.clamp(omega, -HangConstants.kMaxOmega, HangConstants.kMaxOmega);

                if (hasTarget) {
                    // Lateral: center tag in camera frame (yaw → strafe)
                    // Camera yaw positive = target left of center → strafe left (positive vy)
                    double yawDeg = visionSim.getRearTargetYawDegrees();
                    // Convert yaw degrees to approximate lateral offset in meters
                    // At typical distances (~1m), 1 degree ≈ 0.017m lateral offset
                    double lateralError = Math.toRadians(yawDeg) * visionSim.getRearDistanceToTarget();
                    vy = latPID.calculate(lateralError, 0);
                    vy = MathUtil.clamp(vy, -HangConstants.kMaxLateralSpeed, HangConstants.kMaxLateralSpeed);

                    // Distance: reach correct standoff from wall
                    double measuredDistance = visionSim.getRearDistanceToTarget();
                    // Negative output → drive backward (toward wall) since camera faces backward
                    double distOutput = distPID.calculate(measuredDistance, HangConstants.kTargetDistanceFromWall);
                    // Camera faces backward: positive distOutput means "too far" → drive backward
                    // In robot frame, backward is negative X
                    vx = -MathUtil.clamp(distOutput, -HangConstants.kMaxForwardSpeed, HangConstants.kMaxForwardSpeed);
                } else {
                    // No target — only correct rotation, hold position otherwise
                    latPID.reset();
                    distPID.reset();
                }

                // Telemetry
                SmartDashboard.putNumber("Hang/RotErrorDeg", Math.toDegrees(rotPID.getError()));
                SmartDashboard.putNumber("Hang/LatOutput", vy);
                SmartDashboard.putNumber("Hang/DistOutput", vx);
                SmartDashboard.putNumber("Hang/OmegaOutput", omega);

                // Drive with combined robot-relative speeds
                swerve.drive(new ChassisSpeeds(vx, vy, omega));

            }, swerve).until(() ->
                rotPID.atSetpoint() && latPID.atSetpoint() && distPID.atSetpoint()
                && visionSim.rearHasTarget()
            );
        }, java.util.Set.of(swerve));
    }
}
