// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import java.util.function.DoubleSupplier;

import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.utils.FieldGeometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for drive-related commands using SwerveSubsystem from AdambotsLib.
 *
 * <p>This class provides static factory methods that create commands
 * for driving operations. The SwerveSubsystem itself provides many commands,
 * but this factory is for more complex driving sequences or combinations.
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * Buttons.JoystickButton3.onTrue(DriveCommands.turnToAngleCommand(swerve, 90));
 * </pre>
 *
 * <p>Note: SwerveSubsystem from AdambotsLib already provides:
 * <ul>
 *   <li>driveCommand() - Field-oriented drive with joystick</li>
 *   <li>driveToPoseCommand() - Drive to a specific pose</li>
 *   <li>driveForwardDistanceCommand() - Drive forward a distance</li>
 *   <li>centerModulesCommand() - Center all modules</li>
 *   <li>lock() - X-lock wheels to prevent pushing</li>
 *   <li>zeroGyro() - Reset gyro heading</li>
 *   <li>getAutonomousCommand() - PathPlanner auto routines</li>
 * </ul>
 */
public final class DriveCommands {

    // Prevent instantiation
    private DriveCommands() {}

    /**
     * Creates a command that turns the robot to face a specific angle.
     *
     * @param swerve The swerve drive subsystem
     * @param angleDegrees The target angle in degrees (field-relative)
     * @return Command that turns to the specified angle
     */
    public static Command turnToAngleCommand(SwerveSubsystem swerve, double angleDegrees) {
        return swerve.driveToPoseCommand(
            new Pose2d(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(angleDegrees)
            )
        ).withName("TurnToAngle(" + angleDegrees + ")");
    }

    /** Default heading tolerance (degrees) for {@link #backToHubCommand} termination. */
    private static final double kBackToHubToleranceDeg = 2.0;

    /**
     * Pose-trust threshold — below this translation norm (meters from field origin)
     * we treat the pose estimator as uninitialized and refuse to aim. Matches the
     * safety guard in {@code TurretSubsystem.poseTrackCommand}.
     */
    private static final double kPoseTrustRadiusMeters = 1.0;

    /**
     * Pure rotation variant of {@link #backToHubCommand(SwerveSubsystem,
     * DoubleSupplier, DoubleSupplier, double)} — driver translation suppliers default to zero.
     * Terminates once heading is within 2°.
     */
    public static Command backToHubCommand(SwerveSubsystem swerve) {
        return backToHubCommand(swerve, () -> 0.0, () -> 0.0, kBackToHubToleranceDeg);
    }

    /**
     * Rotates the chassis so the robot's back faces the alliance hub, leaving the driver's
     * XY translation under their own control. Intended for the "fixed turret" shot strategy
     * where the shooter is mounted rearward and aiming is done by chassis yaw.
     *
     * <p>Shares geometry with the turret's pose-based tracker
     * ({@link com.adambots.subsystems.TurretSubsystem#poseTrackCommand}): world bearing
     * from robot pose to {@link FieldGeometry#getHubCenter()} via {@code atan2}, flipped by π so
     * the robot's rear — not its front — points at the hub. Omega is computed with YAGSL's
     * {@code swerveController.headingCalculate} (same PID used by heading-lock teleop).
     *
     * <p>If the swerve pose estimator hasn't localized (pose within {@value #kPoseTrustRadiusMeters} m
     * of field origin), the command passes driver translation through with zero rotation and does
     * not terminate — it waits for odometry to become trustworthy before aiming.
     *
     * @param swerve         swerve subsystem
     * @param vxSupplier     driver field-relative X velocity passthrough (m/s)
     * @param vySupplier     driver field-relative Y velocity passthrough (m/s)
     * @param toleranceDeg   terminate when |heading error| is below this (degrees)
     */
    public static Command backToHubCommand(
            SwerveSubsystem swerve,
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            double toleranceDeg) {
        return Commands.run(() -> {
            Pose2d pose = swerve.getPose();
            if (!poseTrusted(pose)) {
                // Odometry hasn't localized — pass driver translation through, skip rotation.
                swerve.driveFieldOriented(new ChassisSpeeds(
                    vxSupplier.getAsDouble(), vySupplier.getAsDouble(), 0));
                return;
            }
            double targetHeadingRad = backToHubTargetHeadingRad(pose);
            double omega = swerve.getSwerveDrive().swerveController.headingCalculate(
                swerve.getHeading().getRadians(), targetHeadingRad);
            swerve.driveFieldOriented(new ChassisSpeeds(
                vxSupplier.getAsDouble(), vySupplier.getAsDouble(), omega));
        }, swerve).until(() -> {
            Pose2d pose = swerve.getPose();
            if (!poseTrusted(pose)) return false;    // stay live until odometry localizes
            double errRad = MathUtil.angleModulus(
                backToHubTargetHeadingRad(pose) - swerve.getHeading().getRadians());
            return Math.abs(Math.toDegrees(errRad)) < toleranceDeg;
        }).withName("BackToHub");
    }

    /**
     * Heading (rad, CCW+) that makes the robot's back face the current alliance hub.
     * atan2 gives the heading that points the robot's FRONT at the hub; the {@code + π}
     * flips it 180° so the rear faces the hub instead.
     */
    private static double backToHubTargetHeadingRad(Pose2d pose) {
        Translation2d hub = FieldGeometry.getHubCenter();
        return Math.atan2(hub.getY() - pose.getY(), hub.getX() - pose.getX()) + Math.PI;
    }

    private static boolean poseTrusted(Pose2d pose) {
        return pose.getTranslation().getNorm() >= kPoseTrustRadiusMeters;
    }
}
