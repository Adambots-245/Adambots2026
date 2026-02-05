// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
// TODO: Import for turret encoder
// import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import com.adambots.lib.actuators.BaseMotor;
// TODO: Import for turret encoder
// import com.adambots.lib.sensors.ThroughBoreEncoder;
import com.adambots.lib.utils.StateMachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
@Logged
public class ShooterSubsystem extends SubsystemBase {

    // ==================== SECTION: HARDWARE ====================
    private final BaseMotor leftMotor;      // Left flywheel (leader)
    private final BaseMotor rightMotor;     // Right flywheel (follower)
    private final BaseMotor turretMotor;    // Turret rotation
    // TODO: Add turret encoder for absolute angle feedback
    // private final ThroughBoreEncoder turretEncoder;  // REV Through Bore (AdambotsLib)

    // ==================== SECTION: STATE ====================
    // TODO: Declare state variables here
    // private double targetVelocity = 0;
    // private double targetTurretAngle = 0;

    // PID Controllers for closed-loop control
    private final PIDController flywheelPID;
    private final SimpleMotorFeedforward flywheelFF;
    private double flywheelSetpoint = 0;

    // Turret position PID
    // The turret PID is controlled via the turretPIDEnabled flag:
    // - false at startup (turret won't move until explicitly commanded)
    // - Enabled by aimTurret() when aiming commands run
    // - Disabled by stopTurret() when stopping or switching to manual/scan mode
    // - When enabled, periodic() runs the PID loop to reach turretSetpointDegrees
    // - When disabled, the turret motor is stopped (no active holding)
    private final PIDController turretPID;
    private double turretSetpointDegrees = 0;
    private boolean turretPIDEnabled = false;

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

    /**
     * Properties for each tracking state.
     *
     * <p>Using an inner class allows state-specific configuration to be passed to actions
     * during state transitions. The StateMachine automatically looks up properties by state
     * when using the Map-based constructor.
     *
     * <p>Example usage in transition:
     * <pre>
     * trackingStateMachine.to(TrackingState.SCANNING)
     *     .executing(props -> {
     *         turretMotor.set(props.turretSpeed());  // Use state-specific speed
     *     })
     *     .request();
     * </pre>
     *
     * @param turretSpeed Motor speed for this state (-1 to 1). Use 0 for states that hold position.
     * @param description Human-readable description for logging/dashboard display.
     */
    public record TrackingStateProperties(
        double turretSpeed,
        String description
        // TODO: Add additional properties as needed, for example:
        // int targetLostThreshold,  // Cycles before transitioning to SCANNING
        // double scanAngleLimit     // Max angle to scan before giving up
    ) {
        /** Creates properties with just a speed (description defaults to empty) */
        public TrackingStateProperties(double turretSpeed) {
            this(turretSpeed, "");
        }
    }

    // State properties map - defines configuration for each tracking state
    // TODO: Adjust these values based on your robot's behavior
    private static final Map<TrackingState, TrackingStateProperties> TRACKING_STATE_PROPERTIES = Map.of(
        TrackingState.DISABLED, new TrackingStateProperties(0.0, "Tracking disabled, holding position"),
        TrackingState.TRACKING, new TrackingStateProperties(0.0, "Actively tracking target"),
        TrackingState.SCANNING, new TrackingStateProperties(0.3, "Scanning for lost target"),  // TODO: Use ShooterConstants.kScanSpeed
        TrackingState.FAILED,   new TrackingStateProperties(0.0, "Scan failed, awaiting manual reset")
    );

    // AdambotsLib StateMachine for tracking behavior
    // StateMachine provides: automatic state transitions, transition listeners, invalid transition checking
    //
    // StateMachine<S, P> where:
    //   S = State enum type (TrackingState)
    //   P = Properties type for each state (TrackingStateProperties)
    //
    // API usage:
    //   trackingStateMachine.to(TrackingState.TRACKING)  // Start a transition
    //       .executing(props -> { /* action to run immediately */ })  // Optional: action on transition
    //       .when(() -> isTargetVisible())  // Optional: condition to complete transition
    //       .request();  // Execute the transition request
    //
    //   trackingStateMachine.getCurrentState()  // Get current state
    //   trackingStateMachine.getTargetProperties()  // Get properties for current target state
    //   trackingStateMachine.isTransitioning()  // Check if mid-transition
    //   trackingStateMachine.periodic()  // Must call in periodic() to complete conditional transitions
    @NotLogged
    private final StateMachine<TrackingState, TrackingStateProperties> trackingStateMachine;

    // Legacy tracking state (kept for reference, will be replaced by StateMachine)
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
        // TODO: Return flywheelPID.atSetpoint() instead of hardcoded false
        // return new Trigger(() -> flywheelPID.atSetpoint());
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
        return new Trigger(() -> turretPIDEnabled && turretPID.atSetpoint());
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
        // Using StateMachine API:
        // TODO: Return true when in TRACKING or SCANNING state
        // return new Trigger(() -> {
        //     var state = trackingStateMachine.getCurrentState();
        //     return state == TrackingState.TRACKING || state == TrackingState.SCANNING;
        // });
        return new Trigger(() -> false);
    }

    /**
     * Returns true when actively tracking a visible target.
     * Useful for LED feedback - green when locked on, yellow when scanning.
     */
    public Trigger isActivelyTrackingTrigger() {
        // TODO: Return true when in TRACKING state AND target is visible
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the turret is scanning for a lost target.
     * Useful for LED feedback or dashboard display.
     */
    public Trigger isScanningTrigger() {
        // Using StateMachine API:
        // TODO: Return true when in SCANNING state
        return new Trigger(() -> false);
    }

    /**
     * Returns true when tracking has failed (scan completed without finding target).
     * Indicates manual intervention is needed.
     */
    public Trigger isTrackingFailedTrigger() {
        // Using StateMachine API:
        // TODO: Return true when in FAILED state
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

    // ==================== SECTION: CONSTRUCTOR ====================
    /**
     * Creates a new ShooterSubsystem.
     *
     * @param leftMotor The left flywheel motor controller (leader)
     * @param rightMotor The right flywheel motor controller (follower)
     * @param turretMotor The turret rotation motor controller
     */
    // TODO: Add turretEncoder parameter to constructor
    // public ShooterSubsystem(BaseMotor leftMotor, BaseMotor rightMotor,
    //                         BaseMotor turretMotor, ThroughBoreEncoder turretEncoder) {
    public ShooterSubsystem(BaseMotor leftMotor, BaseMotor rightMotor, BaseMotor turretMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.turretMotor = turretMotor;
        // TODO: Assign turret encoder
        // this.turretEncoder = turretEncoder;

        // TODO: Configure motor settings
        // leftMotor.setInverted(true);

        // TODO: Configure right motor as follower of left motor
        // rightMotor.setStrictFollower(kShooterLeftMotorPort, true);  // true = inverted

        // TODO: Configure turret soft limits to prevent over-rotation
        // turretMotor.configureSoftLimits(minPosition, maxPosition, true);

        // Initialize flywheel velocity PID
        flywheelPID = new PIDController(
            ShooterConstants.kFlywheelKP,
            ShooterConstants.kFlywheelKI,
            ShooterConstants.kFlywheelKD
        );
        flywheelPID.setTolerance(ShooterConstants.kFlywheelTolerance);

        // Feed-forward compensates for motor physics (kS=0, kV=FF)
        flywheelFF = new SimpleMotorFeedforward(0, ShooterConstants.kFlywheelFF);

        // Initialize turret position PID
        turretPID = new PIDController(
            ShooterConstants.kTurretKP,
            ShooterConstants.kTurretKI,
            ShooterConstants.kTurretKD
        );
        turretPID.setTolerance(ShooterConstants.kTurretAngleTolerance);

        // Initialize tracking state machine with auto-lookup properties map
        // The Map-based constructor automatically loads properties for each state,
        // so you don't need to call .withProperties() on each transition.
        //
        // The logger parameter enables debug output (use Dash::log or null to disable)
        trackingStateMachine = new StateMachine<>(
            TrackingState.DISABLED,
            TRACKING_STATE_PROPERTIES,
            null  // TODO: Replace with Dash::log for debug output
        );

        // TODO: Optionally mark invalid transitions:
        // trackingStateMachine.addInvalidTransition(TrackingState.FAILED, TrackingState.TRACKING,
        //     "Cannot go directly to TRACKING from FAILED - must scan first");

        // TODO: Optionally add transition listeners for logging or side effects:
        // trackingStateMachine.addTransitionListener((from, to) -> {
        //     SmartDashboard.putString("Shooter/TrackingState", to.name());
        //     SmartDashboard.putString("Shooter/StateDescription",
        //         trackingStateMachine.getTargetProperties().description());
        // });
    }

    // ==================== SECTION: COMMAND FACTORIES - FLYWHEEL ====================
    // Commands for controlling the shooter flywheels

    /**
     * Returns a command that spins up the shooter to the default velocity.
     * Uses PID + feed-forward control via flywheelSetpoint (applied in periodic()).
     */
    public Command spinUpCommand() {
        return run(() -> {
            // TODO: Set flywheelSetpoint to the target velocity from Constants
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
        }).withName("SpinUp(" + velocityRPS + ")");
    }

    /**
     * Returns a command that stops the shooter flywheels.
     * Sets flywheelSetpoint to 0, which stops PID control in periodic().
     */
    public Command stopFlywheelCommand() {
        return runOnce(() -> {
            // TODO: Set flywheelSetpoint to 0 to stop the flywheel
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
        }).withName("IdleShooter");
    }

    // ==================== SECTION: COMMAND FACTORIES - TURRET ====================
    // Commands for controlling the turret rotation

    /**
     * Returns a command that rotates the turret to a specific angle.
     * Enables turret PID while running. When the command ends, PID is disabled
     * and the turret stops (no active position holding after command ends).
     *
     * @param angleDegrees Target angle in degrees (0 = forward, positive = left)
     */
    public Command aimTurretCommand(double angleDegrees) {
        return run(() -> {
            aimTurret(angleDegrees);
        })
        .finallyDo(() -> stopTurret())
        .withName("AimTurret(" + angleDegrees + ")");
    }

    /**
     * Returns a command that continuously aims the turret at an angle from a supplier.
     * Use this with vision by passing a supplier that returns the target angle.
     * Enables turret PID while running. When the command ends, PID is disabled
     * and the turret stops (no active position holding after command ends).
     *
     * <p>Example usage with vision:
     * <pre>
     * // In TurretTrackingCommands or RobotContainer:
     * shooter.aimTurretCommand(() -> visionSubsystem.getTargetAngle())
     * </pre>
     *
     * @param angleSupplier Supplies the target angle in degrees (e.g., vision::getTargetAngle)
     */
    public Command aimTurretCommand(java.util.function.DoubleSupplier angleSupplier) {
        return run(() -> {
            aimTurret(angleSupplier.getAsDouble());
        })
        .finallyDo(() -> stopTurret())
        .withName("AimTurretDynamic");
    }

    /**
     * Returns a command that centers the turret (facing forward).
     */
    public Command centerTurretCommand() {
        return aimTurretCommand(0).withName("CenterTurret");
    }

    /**
     * Returns a command that manually rotates the turret.
     * Disables PID control and directly sets motor speed.
     * When the command ends, the turret motor is stopped.
     *
     * @param speed Speed supplier (typically from joystick, -1 to 1)
     */
    public Command manualTurretCommand(java.util.function.DoubleSupplier speed) {
        return run(() -> {
            // Disable PID when in manual mode
            // turretPIDEnabled = false;
            // double adjustedSpeed = speed.getAsDouble() * ShooterConstants.kTurretManualSpeed;
            // turretMotor.set(adjustedSpeed);
        })
        .finallyDo(() -> turretMotor.set(0))
        .withName("ManualTurret");
    }

    /**
     * Returns a command that stops the turret and disables PID control.
     */
    public Command stopTurretCommand() {
        return runOnce(() -> {
            stopTurret();
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


    // ==================== SECTION: COMMAND FACTORIES - TURRET TRACKING ====================
    // Basic turret tracking commands (vision-agnostic primitives)
    // For vision-integrated tracking, use TurretTrackingCommands which coordinates
    // this subsystem with vision suppliers.

    /**
     * Returns a command that stops turret tracking and holds the current position.
     * Use this to disable any active tracking behavior.
     */
    public Command stopTrackingCommand() {
        return runOnce(() -> {
            stopTurret();
            // TODO: Transition state machine to DISABLED state
            // trackingStateMachine.to(TrackingState.DISABLED).request();
        }).withName("StopTracking");
    }

    /**
     * Returns a command that rotates the turret at scan speed.
     * This is a vision-agnostic primitive that just rotates the turret.
     * When the command ends, the turret motor is stopped.
     * Use TurretTrackingCommands for vision-integrated scan-and-reacquire behavior.
     */
    public Command scanTurretCommand() {
        return run(() -> {
            // Disable PID when scanning (open-loop rotation)
            // turretPIDEnabled = false;
            // turretMotor.set(getScanSpeed());
        })
        .finallyDo(() -> turretMotor.set(0))
        .withName("ScanTurret");
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
        double startTime = Timer.getFPGATimestamp();

        // TODO: Update the state machine (required for conditional transitions to complete)
        // trackingStateMachine.periodic();

        // TODO: Run flywheel PID loop when flywheelSetpoint > 0
        // Steps:
        // 1. Get current velocity from motor encoder: double currentVelocity = leftMotor.getVelocity();
        // 2. Calculate PID output: double pidOutput = flywheelPID.calculate(currentVelocity, flywheelSetpoint);
        // 3. Calculate feed-forward: double ffOutput = flywheelFF.calculate(flywheelSetpoint);
        // 4. Apply combined output: leftMotor.setVoltage(pidOutput + ffOutput);
        //
        // Example implementation:
        // if (flywheelSetpoint > 0) {
        //     double currentVelocity = leftMotor.getVelocity().in(RotationsPerSecond);
        //     double pidOutput = flywheelPID.calculate(currentVelocity, flywheelSetpoint);
        //     double ffOutput = flywheelFF.calculate(flywheelSetpoint);
        //     leftMotor.set(pidOutput + ffOutput);
        // }

        // Run turret position PID loop when enabled
        // The turret PID is enabled by aimTurret() and disabled by stopTurret()
        // if (turretPIDEnabled) {
        //     double currentAngle = getTurretAngle();
        //     double pidOutput = turretPID.calculate(currentAngle, turretSetpointDegrees);
        //     turretMotor.set(pidOutput);
        // }

        // TODO: Add telemetry to SmartDashboard
        // SmartDashboard.putNumber("Shooter/FlywheelSetpoint", flywheelSetpoint);
        // SmartDashboard.putNumber("Shooter/FlywheelVelocity", leftMotor.getVelocity());
        // SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelPID.atSetpoint());

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

        Logger.recordOutput("Timing/ShooterSubsystem", (Timer.getFPGATimestamp() - startTime) * 1000.0);
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
        // TODO: Use ThroughBoreEncoder absolute position instead of motor encoder
        // The REV Through Bore Encoder returns position as WPILib Angle (0-360 degrees)
        //
        // double rawAngle = turretEncoder.getPosition().in(Degrees);  // 0.0 to 360.0
        //
        // Apply offset if encoder zero doesn't align with turret forward:
        // double angle = rawAngle - ShooterConstants.kTurretEncoderOffset;
        //
        // Normalize to -180 to +180 range (optional):
        // if (angle > 180) angle -= 360;
        // if (angle < -180) angle += 360;
        // return angle;

        // Fallback: use motor encoder (requires zeroing on startup)
        // return turretMotor.getPosition() * 360.0 / Constants.ShooterConstants.kTurretGearRatio;
        return 0;
    }

    /**
     * Sets the turret to aim at a specific angle.
     * Internal helper used by tracking and manual aim commands.
     * Enables the turret PID and sets the setpoint - the PID loop in periodic() does the control.
     *
     * @param angleDegrees Target angle in degrees (0 = forward)
     */
    private void aimTurret(double angleDegrees) {
        turretSetpointDegrees = angleDegrees;
        turretPIDEnabled = true;
    }

    /**
     * Stops the turret motor and disables the turret PID.
     * Call this when you want to hold position or transition to manual control.
     */
    private void stopTurret() {
        turretPIDEnabled = false;
        turretMotor.set(0);
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
