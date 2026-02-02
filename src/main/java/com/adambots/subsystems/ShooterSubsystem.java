// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.lib.actuators.BaseMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static com.adambots.Constants.*;

import com.adambots.Constants.ShooterConstants;

/**
 * Shooter subsystem for launching game pieces.
 *
 * <p>This subsystem controls:
 * <ul>
 *   <li>Left flywheel motor (leader)</li>
 *   <li>Right flywheel motor (follower - spins opposite direction)</li>
 *   <li>Turret motor for aiming the shooter left/right</li>
 * </ul>
 *
 * <p>AdambotsLib Best Practices:
 * <ul>
 *   <li>Control subsystems using command factories</li>
 *   <li>Get information from subsystems using triggers</li>
 *   <li>Coordinate between subsystems by binding commands to triggers</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor leftMotor;      // Left flywheel (leader)
    private final BaseMotor rightMotor;     // Right flywheel (follower)
    private final BaseMotor turretMotor;    // Turret rotation

    // ==================== SECTION: STATE ====================
    // TODO: Declare state variables here
    // private double targetVelocity = 0;
    // private double targetTurretAngle = 0;

    // ==================== TRACKING STATE MACHINE ====================
    /**
     * States for hub tracking behavior.
     *
     * <p>State machine flow:
     * <pre>
     * DISABLED → TRACKING ↔ SCANNING → FAILED
     *              ↑                      │
     *              └──────────────────────┘ (manual re-enable)
     * </pre>
     */
    public enum TrackingState {
        /** Tracking off, turret holds position */
        DISABLED,
        /** Actively tracking visible target */
        TRACKING,
        /** Lost target, doing one full rotation to reacquire */
        SCANNING,
        /** Scan completed without finding target, waiting for manual reset */
        FAILED
    }

    // Current tracking state
    private TrackingState trackingState = TrackingState.DISABLED;

    // Scan state tracking
    // TODO: Track where the turret was when scan started
    // private double scanStartPosition = 0;

    // TODO: Track how many cycles the target has been lost (for debouncing)
    // private int targetLostCycles = 0;

    // ==================== SECTION: TRIGGERS ====================
    // Expose state as yes/no questions via Trigger objects

    /**
     * Returns true when the shooter flywheels are at the target velocity (ready to shoot).
     */
    public Trigger isAtSpeedTrigger() {
        // TODO: Check if flywheel is at target velocity within tolerance
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the shooter flywheels are spinning (non-zero velocity).
     */
    public Trigger isSpinningTrigger() {
        // TODO: Check if flywheel is spinning
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the shooter is idle (stopped).
     */
    public Trigger isIdleTrigger() {
        // TODO: Check if flywheel is stopped
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is at the target angle.
     */
    public Trigger isTurretAtTargetTrigger() {
        // TODO: Check if turret position is within tolerance of target
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is centered (facing forward).
     */
    public Trigger isTurretCenteredTrigger() {
        // TODO: Check if turret is at center position
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is at the left limit.
     */
    public Trigger isTurretAtLeftLimitTrigger() {
        // TODO: Check if turret is at left soft limit
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is at the right limit.
     */
    public Trigger isTurretAtRightLimitTrigger() {
        // TODO: Check if turret is at right soft limit
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the shooter is ready to fire (at speed and turret aimed).
     */
    public Trigger isReadyToFireTrigger() {
        // TODO: Combine isAtSpeedTrigger and isTurretAtTargetTrigger
        return new Trigger(() -> false);
    }

    /**
     * Returns true when hub tracking mode is enabled (TRACKING or SCANNING state).
     * When enabled, the turret will continuously orient toward the hub target
     * or scan to find it.
     */
    public Trigger isTrackingEnabledTrigger() {
        // TODO: Return true when in TRACKING or SCANNING state
        // return new Trigger(() -> trackingState == TrackingState.TRACKING
        //     || trackingState == TrackingState.SCANNING);
        return new Trigger(() -> false);
    }

    /**
     * Returns true when actively tracking a visible target.
     * Useful for LED feedback - green when locked on, yellow when scanning.
     */
    public Trigger isActivelyTrackingTrigger() {
        // TODO: Return true when in TRACKING state AND target is visible
        // return new Trigger(() -> trackingState == TrackingState.TRACKING
        //     && isTargetVisible());
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is scanning for a lost target.
     * Useful for LED feedback or dashboard display.
     */
    public Trigger isScanningTrigger() {
        // TODO: Return true when in SCANNING state
        // return new Trigger(() -> trackingState == TrackingState.SCANNING);
        return new Trigger(() -> false);
    }

    /**
     * Returns true when tracking has failed (scan completed without finding target).
     * Indicates manual intervention is needed.
     */
    public Trigger isTrackingFailedTrigger() {
        // TODO: Return true when in FAILED state
        // return new Trigger(() -> trackingState == TrackingState.FAILED);
        return new Trigger(() -> false);
    }

    /**
     * Returns true when hub tracking is enabled AND the turret is on target.
     * Useful for determining when it's safe to shoot while tracking.
     */
    public Trigger isTrackingAndOnTargetTrigger() {
        // TODO: Combine isActivelyTrackingTrigger and isTurretAtTargetTrigger
        // return isActivelyTrackingTrigger().and(isTurretAtTargetTrigger());
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the hub target is visible to the vision system.
     * Used to determine if tracking is possible.
     */
    public Trigger isTargetVisibleTrigger() {
        // TODO: Check if vision system can see the hub target
        // return new Trigger(() -> vision.hasTarget());
        return new Trigger(() -> false);
    }

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new ShooterSubsystem.
     *
     * @param leftMotor The left flywheel motor controller (leader)
     * @param rightMotor The right flywheel motor controller (follower)
     * @param turretMotor The turret rotation motor controller
     */
    public ShooterSubsystem(BaseMotor leftMotor, BaseMotor rightMotor, BaseMotor turretMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.turretMotor = turretMotor;

        // TODO: Configure motor settings
        // leftMotor.setInverted(true);

        // TODO: Configure right motor as follower of left motor
        // rightMotor.setStrictFollower(kShooterLeftMotorPort, true);  // true = inverted

        // TODO: Configure turret soft limits to prevent over-rotation
        // turretMotor.configureSoftLimits(minPosition, maxPosition, true);
    }

    // ==================== SECTION: COMMAND FACTORIES - FLYWHEEL ====================
    // Commands for controlling the shooter flywheels

    /**
     * Returns a command that spins up the shooter to the default velocity.
     * TODO: Set target velocity in Constants
     */
    public Command spinUpCommand() {
        return run(() -> {
            // TODO: Set flywheel to target velocity using velocity control
            // leftMotor.setVelocity(Constants.ShooterConstants.kDefaultVelocity);
            // Note: Right motor follows automatically if configured as follower
        }).withName("SpinUp");
    }

    /**
     * Returns a command that spins up the shooter to a specific velocity.
     *
     * @param velocityRPS Target velocity in rotations per second
     */
    public Command spinUpCommand(double velocityRPS) {
        return run(() -> {
            // TODO: Set flywheel to specified velocity
            // leftMotor.setVelocity(velocityRPS);
        }).withName("SpinUp(" + velocityRPS + ")");
    }

    /**
     * Returns a command that stops the shooter flywheels.
     */
    public Command stopFlywheelCommand() {
        return runOnce(() -> {
            // TODO: Stop the flywheel motors
            // leftMotor.set(0);
            // If not using follower mode, also stop right motor
            // rightMotor.set(0);
        }).withName("StopFlywheel");
    }

    /**
     * Returns a command that spins up and waits until at speed.
     * Useful for autonomous or pre-spinning before a shot.
     */
    public Command spinUpAndWaitCommand() {
        return spinUpCommand()
            .until(isAtSpeedTrigger())
            .withName("SpinUpAndWait");
    }

    /**
     * Returns a command that idles the shooter at a low speed.
     * Keeps the flywheel warm for faster spin-up.
     */
    public Command idleCommand() {
        return run(() -> {
            // TODO: Set flywheel to idle velocity
            // leftMotor.set(Constants.ShooterConstants.kIdleSpeed);
        }).withName("IdleShooter");
    }

    // ==================== SECTION: COMMAND FACTORIES - TURRET ====================
    // Commands for controlling the turret rotation

    /**
     * Returns a command that rotates the turret to a specific angle.
     *
     * @param angleDegrees Target angle in degrees (0 = forward, positive = left)
     */
    public Command aimTurretCommand(double angleDegrees) {
        return run(() -> {
            // TODO: Set turret to target angle using position control
            // double targetPosition = angleDegrees / 360.0 * Constants.ShooterConstants.kTurretGearRatio;
            // turretMotor.setPosition(targetPosition);
        }).withName("AimTurret(" + angleDegrees + ")");
    }

    /**
     * Returns a command that centers the turret (facing forward).
     */
    public Command centerTurretCommand() {
        return aimTurretCommand(0).withName("CenterTurret");
    }

    /**
     * Returns a command that manually rotates the turret.
     *
     * @param speed Speed supplier (typically from joystick, -1 to 1)
     */
    public Command manualTurretCommand(java.util.function.DoubleSupplier speed) {
        return run(() -> {
            // TODO: Set turret motor to supplied speed with limits check
            // double adjustedSpeed = speed.getAsDouble() * Constants.ShooterConstants.kTurretManualSpeed;
            // turretMotor.set(adjustedSpeed);
        }).withName("ManualTurret");
    }

    /**
     * Returns a command that stops the turret.
     */
    public Command stopTurretCommand() {
        return runOnce(() -> {
            // TODO: Stop the turret motor
            // turretMotor.set(0);
        }).withName("StopTurret");
    }

    /**
     * Returns a command that aims the turret and waits until on target.
     *
     * @param angleDegrees Target angle in degrees
     */
    public Command aimAndWaitCommand(double angleDegrees) {
        return aimTurretCommand(angleDegrees)
            .until(isTurretAtTargetTrigger())
            .withName("AimAndWait");
    }

    /**
     * Returns a command that tracks a target using vision.
     * Requires integration with vision subsystem.
     * Note: This command tracks regardless of the tracking flag - use for one-shot tracking.
     */
    public Command trackTargetCommand() {
        return run(() -> {
            // TODO: Get target angle from vision and aim turret
            // double targetAngle = getTargetAngleFromVision();
            // aimTurret(targetAngle);
        }).withName("TrackTarget");
    }

    // ==================== SECTION: COMMAND FACTORIES - HUB TRACKING ====================
    // Commands for continuous hub tracking mode with scan-to-reacquire
    // When enabled, the turret automatically orients toward the hub target using vision
    // If target is lost, performs one full scan rotation to reacquire

    /**
     * Returns a command that enables hub tracking mode.
     * If target is visible, goes to TRACKING state.
     * If target is NOT visible, goes to SCANNING state to find it.
     *
     * <p>Usage in RobotContainer:
     * <pre>
     * Buttons.XboxLeftBumper.onTrue(shooter.enableTrackingCommand());
     * </pre>
     */
    public Command enableTrackingCommand() {
        return runOnce(() -> {
            // TODO: Check if target is visible to determine starting state
            // if (isTargetVisible()) {
            //     trackingState = TrackingState.TRACKING;
            // } else {
            //     // Target not visible - start scanning to find it
            //     trackingState = TrackingState.SCANNING;
            //     scanStartPosition = getTurretAngle();  // Record start position
            // }
            // targetLostCycles = 0;
            //
            // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
        }).withName("EnableTracking");
    }

    /**
     * Returns a command that disables hub tracking mode.
     * The turret will hold its current position when disabled.
     */
    public Command disableTrackingCommand() {
        return runOnce(() -> {
            // TODO: Set state to DISABLED and stop turret
            // trackingState = TrackingState.DISABLED;
            // targetLostCycles = 0;
            // turretMotor.set(0);
            //
            // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
        }).withName("DisableTracking");
    }

    /**
     * Returns a command that toggles hub tracking mode on/off.
     * Convenient for binding to a single button.
     */
    public Command toggleTrackingCommand() {
        return runOnce(() -> {
            // TODO: Toggle between DISABLED and enabled states
            // if (trackingState == TrackingState.DISABLED) {
            //     // Enable - same logic as enableTrackingCommand
            //     if (isTargetVisible()) {
            //         trackingState = TrackingState.TRACKING;
            //     } else {
            //         trackingState = TrackingState.SCANNING;
            //         scanStartPosition = getTurretAngle();
            //     }
            // } else {
            //     trackingState = TrackingState.DISABLED;
            //     turretMotor.set(0);
            // }
            // targetLostCycles = 0;
            //
            // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
        }).withName("ToggleTracking");
    }

    /**
     * Returns a command that manually triggers a scan for the target.
     * Useful after FAILED state to try again, or when robot moves to new location.
     *
     * <p>Usage in RobotContainer:
     * <pre>
     * Buttons.XboxYButton.onTrue(shooter.scanForTargetCommand());
     * </pre>
     */
    public Command scanForTargetCommand() {
        return runOnce(() -> {
            // TODO: Start a new scan
            // trackingState = TrackingState.SCANNING;
            // scanStartPosition = getTurretAngle();  // Record current position
            // targetLostCycles = 0;
            //
            // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
        }).withName("ScanForTarget");
    }

    /**
     * Returns a command that continuously manages hub tracking via state machine.
     * This handles all tracking states: DISABLED, TRACKING, SCANNING, and FAILED.
     *
     * <p>State machine behavior:
     * <ul>
     *   <li>DISABLED: Hold turret position, do nothing</li>
     *   <li>TRACKING: Aim at visible target; if lost for N cycles, transition to SCANNING</li>
     *   <li>SCANNING: Rotate turret slowly; if target found, go to TRACKING; if full rotation done, go to FAILED</li>
     *   <li>FAILED: Hold position, wait for manual intervention (enableTracking or scanForTarget)</li>
     * </ul>
     *
     * <p>This is intended to be used as the DEFAULT COMMAND for the shooter subsystem.
     *
     * <p>Usage in RobotContainer:
     * <pre>
     * // Set as default command (always runs when nothing else needs turret)
     * shooter.setDefaultCommand(shooter.autoTrackCommand());
     *
     * // Bind enable/disable to buttons
     * Buttons.XboxLeftBumper.onTrue(shooter.enableTrackingCommand());
     * Buttons.XboxRightBumper.onTrue(shooter.disableTrackingCommand());
     *
     * // Manual scan button (useful after FAILED state)
     * Buttons.XboxYButton.onTrue(shooter.scanForTargetCommand());
     * </pre>
     */
    public Command autoTrackCommand() {
        return run(() -> {
            // TODO: Implement state machine logic
            //
            // switch (trackingState) {
            //     case DISABLED:
            //         // Hold position - do nothing (or optionally stop motor)
            //         // turretMotor.set(0);
            //         break;
            //
            //     case TRACKING:
            //         if (isTargetVisible()) {
            //             // Target visible - aim at it
            //             // double targetAngle = getTargetAngleFromVision();
            //             // aimTurret(targetAngle);
            //             // targetLostCycles = 0;  // Reset lost counter
            //         } else {
            //             // Target not visible - increment lost counter
            //             // targetLostCycles++;
            //             //
            //             // // Check if we've lost target long enough to start scanning
            //             // if (targetLostCycles >= Constants.ShooterConstants.kTargetLostCycles) {
            //             //     trackingState = TrackingState.SCANNING;
            //             //     scanStartPosition = getTurretAngle();
            //             //     SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
            //             // }
            //             //
            //             // // While waiting, hold current position
            //             // turretMotor.set(0);
            //         }
            //         break;
            //
            //     case SCANNING:
            //         if (isTargetVisible()) {
            //             // Found target during scan - transition to TRACKING
            //             // trackingState = TrackingState.TRACKING;
            //             // targetLostCycles = 0;
            //             // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
            //         } else if (hasCompletedFullRotation()) {
            //             // Completed full scan without finding target - give up
            //             // trackingState = TrackingState.FAILED;
            //             // turretMotor.set(0);
            //             // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
            //         } else {
            //             // Continue scanning - rotate turret slowly
            //             // turretMotor.set(getScanSpeed());
            //         }
            //         break;
            //
            //     case FAILED:
            //         // Scan failed - hold position and wait for manual intervention
            //         // turretMotor.set(0);
            //         // User must call enableTrackingCommand() or scanForTargetCommand() to retry
            //         break;
            // }
        }).withName("AutoTrack");
    }

    /**
     * Returns a command that enables tracking, waits until on target, then disables.
     * Useful for one-shot aim-and-lock behavior.
     */
    public Command trackUntilOnTargetCommand() {
        return enableTrackingCommand()
            .andThen(autoTrackCommand().until(isTurretAtTargetTrigger()))
            .andThen(disableTrackingCommand())
            .withName("TrackUntilOnTarget");
    }

    // ==================== SECTION: COMMAND FACTORIES - COMBINED ====================
    // Commands that coordinate flywheel and turret

    /**
     * Returns a command that prepares to shoot: spins up and aims.
     *
     * @param angleDegrees Turret angle in degrees
     */
    public Command prepareToShootCommand(double angleDegrees) {
        return run(() -> {
            // TODO: Spin up flywheel and aim turret simultaneously
            // leftMotor.setVelocity(Constants.ShooterConstants.kDefaultVelocity);
            // aimTurret(angleDegrees);
        }).withName("PrepareToShoot");
    }

    /**
     * Returns a command that stops all shooter motors.
     */
    public Command stopAllCommand() {
        return runOnce(() -> {
            // TODO: Stop both flywheel and turret
            // leftMotor.set(0);
            // rightMotor.set(0);  // If not using follower
            // turretMotor.set(0);
        }).withName("StopAll");
    }

    // ==================== SECTION: PERIODIC ====================
    /**
     * This method runs every 20ms.
     * Use for: caching sensor values, updating odometry, logging/telemetry.
     * Do NOT use for: control logic, state machines, conditional actions.
     */
    @Override
    public void periodic() {
        // TODO: Cache velocity values
        // double leftVelocity = leftMotor.getVelocity();
        // double rightVelocity = rightMotor.getVelocity();
        // double turretPosition = turretMotor.getPosition();

        // TODO: Update telemetry
        // SmartDashboard.putNumber("Shooter/LeftVelocity", leftVelocity);
        // SmartDashboard.putNumber("Shooter/RightVelocity", rightVelocity);
        // SmartDashboard.putNumber("Shooter/TurretPosition", turretPosition);
        // SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeedTrigger().getAsBoolean());
        // SmartDashboard.putBoolean("Shooter/TurretOnTarget", isTurretAtTargetTrigger().getAsBoolean());
        // SmartDashboard.putString("Shooter/TrackingState", trackingState.name());
        // SmartDashboard.putBoolean("Shooter/TargetVisible", isTargetVisibleTrigger().getAsBoolean());
    }

    // ==================== SECTION: PRIVATE HELPERS ====================
    // Internal helper methods

    /**
     * Gets the current flywheel velocity.
     *
     * @return The average velocity of both flywheels in RPS
     */
    private double getFlywheelVelocity() {
        // TODO: Return average flywheel velocity
        // return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;
        return 0;
    }

    /**
     * Gets the current turret angle in degrees.
     *
     * @return The turret angle (0 = forward, positive = left)
     */
    private double getTurretAngle() {
        // TODO: Convert motor position to degrees
        // return turretMotor.getPosition() * 360.0 / Constants.ShooterConstants.kTurretGearRatio;
        return 0;
    }

    /**
     * Gets the target angle from the vision system.
     * This method should query PhotonVision or Limelight for the hub target.
     *
     * @return The angle to the hub target in degrees, or 0 if no target visible
     */
    private double getTargetAngleFromVision() {
        // TODO: Implement vision integration
        // Option 1: Using PhotonVision
        // var result = photonCamera.getLatestResult();
        // if (result.hasTargets()) {
        //     return result.getBestTarget().getYaw();
        // }
        //
        // Option 2: Using Limelight NetworkTables
        // double tx = NetworkTableInstance.getDefault()
        //     .getTable("limelight").getEntry("tx").getDouble(0);
        // return tx;
        //
        // Option 3: Using AdambotsLib vision (if integrated with swerve)
        // Get target pose from swerve vision and calculate angle
        return 0;
    }

    /**
     * Checks if the vision system can see the hub target.
     *
     * @return true if a valid target is visible
     */
    private boolean isTargetVisible() {
        // TODO: Implement target visibility check
        // Option 1: Using PhotonVision
        // return photonCamera.getLatestResult().hasTargets();
        //
        // Option 2: Using Limelight
        // return NetworkTableInstance.getDefault()
        //     .getTable("limelight").getEntry("tv").getDouble(0) == 1;
        return false;
    }

    /**
     * Sets the turret to aim at a specific angle.
     * Internal helper used by tracking and manual aim commands.
     *
     * @param angleDegrees Target angle in degrees (0 = forward)
     */
    private void aimTurret(double angleDegrees) {
        // TODO: Convert angle to motor position and set
        // double targetPosition = angleDegrees / 360.0 * Constants.ShooterConstants.kTurretGearRatio;
        // turretMotor.setPosition(targetPosition);
    }

    // ==================== SCAN SUPPORT METHODS ====================

    /**
     * Checks if the turret has completed a full rotation from the scan start position.
     * Used by the SCANNING state to know when to give up.
     *
     * @return true if turret has rotated 360 degrees from scan start
     */
    private boolean hasCompletedFullRotation() {
        // TODO: Calculate if turret has completed full rotation
        // double currentAngle = getTurretAngle();
        // double rotationAmount = Math.abs(currentAngle - scanStartPosition);
        //
        // // Handle wraparound (turret may have crossed 0/360 boundary)
        // // For simplicity, just check if we've moved >= 360 degrees
        // return rotationAmount >= Constants.ShooterConstants.kFullRotationDegrees;
        return false;
    }

    /**
     * Gets the scan rotation speed from constants.
     * The turret rotates at this speed during SCANNING state.
     *
     * @return Motor power for scanning (-1 to 1)
     */
    private double getScanSpeed() {
        // TODO: Return scan speed from constants
        // return Constants.ShooterConstants.kScanSpeed;
        return 0.3;  // Default placeholder
    }
}
