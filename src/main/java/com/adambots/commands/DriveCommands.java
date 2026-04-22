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

    /** Default heading tolerance (degrees) for hub-aim command termination. */
    private static final double kHubAimToleranceDeg = 2.0;

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
        return backToHubCommand(swerve, () -> 0.0, () -> 0.0, kHubAimToleranceDeg);
    }

    /**
     * Rotates the chassis so the robot's <b>back</b> faces the alliance hub, leaving the
     * driver's XY translation under their own control. Intended for the "fixed turret" shot
     * strategy where the shooter is mounted rearward and aiming is done by chassis yaw.
     *
     * <p>See {@link #aimChassisAtHubCommand} for the shared implementation + pose-trust guard.
     * Flipped 180° ({@code Math.PI} offset) from {@link #frontToHubCommand}.
     */
    public static Command backToHubCommand(
            SwerveSubsystem swerve,
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            double toleranceDeg) {
        return aimChassisAtHubCommand(swerve, vxSupplier, vySupplier, toleranceDeg,
                                      Math.PI, "BackToHub");
    }

    /**
     * Pure rotation variant of {@link #frontToHubCommand(SwerveSubsystem,
     * DoubleSupplier, DoubleSupplier, double)} — driver translation suppliers default to zero.
     * Terminates once heading is within 2°.
     */
    public static Command frontToHubCommand(SwerveSubsystem swerve) {
        return frontToHubCommand(swerve, () -> 0.0, () -> 0.0, kHubAimToleranceDeg);
    }

    /**
     * Rotates the chassis so the robot's <b>front</b> faces the alliance hub. Mirror of
     * {@link #backToHubCommand} for the forward-mounted shooter strategy (fixed turret,
     * shooter facing the robot's front).
     *
     * <p>See {@link #aimChassisAtHubCommand} for the shared implementation + pose-trust guard.
     * Uses an angular offset of 0 rad (no flip — atan2 result is the target heading directly).
     */
    public static Command frontToHubCommand(
            SwerveSubsystem swerve,
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            double toleranceDeg) {
        return aimChassisAtHubCommand(swerve, vxSupplier, vySupplier, toleranceDeg,
                                      0.0, "FrontToHub");
    }

    /**
     * Shared command factory for chassis-aims-at-hub commands. Computes the heading
     * from robot pose to {@link FieldGeometry#getHubCenter()} via {@code atan2}, adds
     * the caller's {@code headingOffsetRad} (0 for front-at-hub, π for back-at-hub),
     * and drives omega through YAGSL's {@code swerveController.headingCalculate} (same
     * PID used by heading-lock teleop).
     *
     * <p>If the swerve pose estimator hasn't localized (pose within
     * {@value #kPoseTrustRadiusMeters} m of field origin), the command passes driver
     * translation through with zero rotation and does not terminate — it waits for
     * odometry to become trustworthy before aiming.
     *
     * @param swerve             swerve subsystem
     * @param vxSupplier         driver field-relative X velocity passthrough (m/s)
     * @param vySupplier         driver field-relative Y velocity passthrough (m/s)
     * @param toleranceDeg       terminate when |heading error| is below this (degrees)
     * @param headingOffsetRad   added to the atan2 result to select front- vs rear-facing
     * @param name               command name (for logs / AdvantageScope)
     */
    private static Command aimChassisAtHubCommand(
            SwerveSubsystem swerve,
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            double toleranceDeg,
            double headingOffsetRad,
            String name) {
        return Commands.run(() -> {
            Pose2d pose = swerve.getPose();
            if (!poseTrusted(pose)) {
                // Odometry hasn't localized — pass driver translation through, skip rotation.
                swerve.driveFieldOriented(new ChassisSpeeds(
                    vxSupplier.getAsDouble(), vySupplier.getAsDouble(), 0));
                return;
            }
            double targetHeadingRad = hubAimHeadingRad(pose, headingOffsetRad);
            double omega = swerve.getSwerveDrive().swerveController.headingCalculate(
                swerve.getHeading().getRadians(), targetHeadingRad);
            swerve.driveFieldOriented(new ChassisSpeeds(
                vxSupplier.getAsDouble(), vySupplier.getAsDouble(), omega));
        }, swerve).until(() -> {
            Pose2d pose = swerve.getPose();
            if (!poseTrusted(pose)) return false;    // stay live until odometry localizes
            double errRad = MathUtil.angleModulus(
                hubAimHeadingRad(pose, headingOffsetRad) - swerve.getHeading().getRadians());
            return Math.abs(Math.toDegrees(errRad)) < toleranceDeg;
        }).withName(name);
    }

    /**
     * Target heading (rad, CCW+) that aims the chassis at the current alliance hub.
     * {@code offsetRad = 0} → front faces hub; {@code offsetRad = π} → back faces hub.
     */
    private static double hubAimHeadingRad(Pose2d pose, double offsetRad) {
        Translation2d hub = FieldGeometry.getHubCenter();
        return Math.atan2(hub.getY() - pose.getY(), hub.getX() - pose.getX()) + offsetRad;
    }

    private static boolean poseTrusted(Pose2d pose) {
        return pose.getTranslation().getNorm() >= kPoseTrustRadiusMeters;
    }
}
