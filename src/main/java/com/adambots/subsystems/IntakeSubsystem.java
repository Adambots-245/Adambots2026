package com.adambots.subsystems;

import static com.adambots.logging.LogUtil.ESSENTIAL;
import static com.adambots.logging.LogUtil.DIAGNOSTIC;
import static com.adambots.logging.LogUtil.log;

import com.adambots.Constants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.SimConstants;
import com.adambots.Robot;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.utils.Dash;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

/**
 * Intake subsystem using TalonFX onboard PID with gravity compensation.
 *
 * <p>
 * The arm motor runs Motion Magic with Arm_Cosine gravity feedforward at 1kHz
 * on the motor controller, providing stable holding and smooth motion.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final BaseMotor intakeMotor;
    private final BaseMotor intakeArmMotor;

    // Simulation
    private SingleJointedArmSim armSim;
    private Mechanism2d mech2d;
    private MechanismLigament2d armLigament;
    private double simMotorVoltage;
    private double simArmAngleDeg;

    private double armLoweredPosition = IntakeConstants.kArmLoweredPosition;
    private double armRaisedPosition = IntakeConstants.kArmRaisedPosition;

    // Coast-when-lowered: arm goes loose at lowered position for compliance
    private boolean armInCoast = false;

    // Roller jam detection state
    private boolean rollerReversing = false;
    private boolean wasRollerRunningLastCycle = false;
    private final Timer rollerJamTimer = new Timer();

    // Cached PID gains for simulation voltage computation (updated by setArmPID)
    private double simP = IntakeConstants.kArmP;
    private double simD = IntakeConstants.kArmD;
    private double simKG = IntakeConstants.kArmKG;
    private double simKS = IntakeConstants.kArmKS;

    /**
     * Returns true when the intake roller is running (velocity above threshold).
     */
    public Trigger isRunningTrigger() {
        return new Trigger(() -> Math
                .abs(intakeMotor.getVelocity().in(RotationsPerSecond)) > IntakeConstants.kRollerRunningThreshold);
    }

    private double targetPosition = 0;

    public IntakeSubsystem(BaseMotor intakeMotor, BaseMotor intakeArmMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeArmMotor = intakeArmMotor;

        configureMotors();

        // No manual seeding needed — the throughbore is wired directly to the
        // TalonFXS data port and configured as the closed-loop feedback source,
        // so getPosition() already reflects the real arm angle at boot.

        if (Constants.INTAKE_TAB) {
            setupDash();
        }

        if (Robot.isSimulation()) {
            setupSimulation();
        }
    }

    private void configureMotors() {
        intakeMotor.configure()
                .brakeMode(false)
                .currentLimits(IntakeConstants.kRollerStatorCurrentLimit, IntakeConstants.kRollerSupplyCurrentLimit,
                        3500)
                .apply();

        // Configure arm motor: brake mode + current limits + gear ratio
        intakeArmMotor.configure()
                .brakeMode(true)
                .inverted(true)
                .currentLimits(IntakeConstants.kArmStatorCurrentLimit, IntakeConstants.kArmSupplyCurrentLimit, 2500)
                .gravity(BaseMotor.GravityType.ARM_COSINE)
                .sensorToMechanismRatio(IntakeConstants.kArmTotalGearRatio)
                .motionMagic(
                        RotationsPerSecond.of(IntakeConstants.kArmCruiseVelocity),
                        RotationsPerSecondPerSecond.of(IntakeConstants.kArmAcceleration),
                        IntakeConstants.kArmJerk)
                .apply();

        // intakeArmMotor.configureHardLimits(false, true, 0, 0);

        // Throughbore is hardwired to the TalonFXS data port and used as the
        // closed-loop feedback source. The FXS reads the PWM signal directly
        // at 1kHz — no DIO polling, no re-sync, no rotor/sensor drift.
        //
        // Args: sensorToMechanismRatio=1.0 (throughbore is on the mechanism
        // side, post-gearbox), absoluteSensorOffset=0.0, discontinuityPoint=1.0,
        // opposeMotor=true (phase-flipped so the motor and sensor agree on
        // direction — fixed the "slams into stop" behavior we hit initially).
        intakeArmMotor.configureExternalPulseWidthSensor(1.0, 0.0, 1.0, true);

        // Enable ContinuousWrap: Phoenix 6 will always take the shortest path
        // to any commanded target, treating the sensor as continuous within 1
        // rotation. This solves the multi-turn-accumulator wrap problem —
        // if the accumulator drifts across power cycles (e.g. the raw sensor
        // wraps through 0/1 inside the arm's range), Motion Magic still moves
        // the short way to the physical lowered/raised position.
        //
        // This is CTRE's built-in solution; see:
        //   https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/
        //     device-specific/talonfx/closed-loop-requests.html
        // (search for "ContinuousWrap"). Works because our arm range (~105°)
        // is well within 1 rotation, so the short path is always the correct
        // physical direction.
        intakeArmMotor.configureContinuousWrap(true);

        // Set extended PID with feedforward gains (kV, kS, kA, kG)
        intakeArmMotor.setPID(0,
                IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD,
                IntakeConstants.kArmKV, IntakeConstants.kArmKS, IntakeConstants.kArmKA,
                IntakeConstants.kArmKG);

        // Re-apply gravity type AFTER setPID — setPID overwrites Slot0Configs
        // which can reset GravityType to the default (Elevator_Static)
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);

        // Soft limits: firmware-level safety rail computed once at boot from
        // the lowered/raised constants plus margin. If the reported position
        // ever drifts beyond these thresholds, the motor controller cuts
        // output in that direction — protects against runaway scenarios
        // (stale constants, slipped sensor mounting, accumulator confusion).
        // The margin lets the arm reach its physical stops normally; the
        // soft limit only triggers when position is way outside the expected
        // range. Note: runtime changes to the lowered/raised tunables do NOT
        // update these thresholds — soft limits are deliberately anchored to
        // the compile-time constants as a static safety rail.
        double lo = Math.min(IntakeConstants.kArmLoweredPosition, IntakeConstants.kArmRaisedPosition);
        double hi = Math.max(IntakeConstants.kArmLoweredPosition, IntakeConstants.kArmRaisedPosition);
        double margin = IntakeConstants.kArmSoftLimitMarginDeg;
        intakeArmMotor.configureSoftLimits(
                (hi + margin) / 360.0,
                (lo - margin) / 360.0,
                true);
    }

    /**
     * Called by {@code RobotContainer.onDisabledInit()}. Replaces any
     * latched Motion Magic control request with {@code DutyCycleOut(0)}
     * so that stale lower/raise/bop commands don't resume driving the arm
     * the moment the robot is re-enabled.
     *
     * <p>Phoenix 6 motor controllers keep the last control request latched
     * in firmware across disable/enable cycles — disable only forces the
     * output to zero temporarily. Issuing {@code set(0)} (which maps to
     * {@code DutyCycleOut(0)}) displaces the stale request so that
     * re-enable is a clean state.
     *
     * <p>Also restores brake mode (in case coast-while-intaking was active)
     * and stops the roller motor.
     */
    public void onDisable() {
        restoreBrakeMode();
        intakeArmMotor.set(0);
        intakeMotor.set(0);
    }

    private void setupDash() {
        Dash.useTab("Intake");

        // Row 0: Telemetry
        Dash.add("Roller Speed", () -> intakeMotor.getVelocity().in(RotationsPerSecond), 0, 0);
        Dash.add("Arm Speed", () -> intakeArmMotor.getVelocity().in(RotationsPerSecond), 2, 0);
        Dash.add("Arm Encoder (deg)", () -> intakeArmMotor.getPosition() * 360.0, 3, 0);
        Dash.add("Arm Mech Pos (rot)", () -> intakeArmMotor.getPosition(), 4, 0);
        Dash.add("Arm Target (mech rot)", () -> targetPosition, 5, 0);
        Dash.add("Target (deg)", () -> targetPosition * 360.0, 6, 0);
        Dash.add("Roller Current", () -> intakeMotor.getCurrentDraw().in(Amps), 10, 0);
        Dash.add("Arm Current", () -> intakeArmMotor.getCurrentDraw().in(Amps), 11, 0);

        // Row 0 (cont.): Sim diagnostics (only meaningful in simulation)
        Dash.add("Sim Voltage", () -> simMotorVoltage, 12, 0);
        Dash.add("Sim Angle Deg", () -> simArmAngleDeg, 13, 0);

        // Row 1: Commands
        Dash.addCommand("Start Intake", runIntakeCommand(), 0, 1);
        Dash.addCommand("Reverse Intake", reverseIntakeCommand(), 1, 1);
        Dash.addCommand("Stop Intake", stopIntakeCommand(), 2, 1);
        Dash.addCommand("Lower Arm", runLowerIntakeArmCommand(), 3, 1);
        Dash.addCommand("Raise Arm", runRaiseIntakeArmCommand(), 4, 1);
        Dash.addCommand("Stop Arm", stopIntakeArmCommand(), 5, 1);
        Dash.addCommand("Bop Arm", bopArmCommand(), 6, 1);

        Dash.useDefaultTab();
    }

    // ==================== Tuning Setters (called by TuningManager)
    // ====================

    public void setArmPID(double p, double i, double d, double kG, double kS, double kV, double kA) {
        intakeArmMotor.setPID(0, p, i, d, kV, kS, kA, kG);
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
        simP = p;
        simD = d;
        simKG = kG;
        simKS = kS;
    }

    public void setMotionMagic(double cruiseVel, double accel) {
        intakeArmMotor.configure()
                .motionMagic(
                        RotationsPerSecond.of(cruiseVel),
                        RotationsPerSecondPerSecond.of(accel),
                        IntakeConstants.kArmJerk)
                .apply();
    }

    public void setArmLoweredPosition(double pos) {
        armLoweredPosition = pos;
    }

    public void setArmRaisedPosition(double pos) {
        armRaisedPosition = pos;
    }

    /**
     * Run the intake roller at the configured speed.
     */
    public void runIntake() {
        if (!rollerReversing) {
            intakeMotor.set(IntakeConstants.kIntakeSpeed);
        }
        // Coast arm for compliance during active intake
        if (!armInCoast) {
            intakeArmMotor.setBrakeMode(false);
            armInCoast = true;
        }
    }

    /**
     * Run the intake roller in reverse.
     */
    public void reverseIntake() {
        intakeMotor.set(-IntakeConstants.kIntakeSpeed);
    }

    /**
     * Stop the intake roller motor.
     */
    public void stopIntake() {
        intakeMotor.set(0);
        rollerReversing = false;
        restoreBrakeMode();
    }

    /**
     * Lower the intake arm using onboard Motion Magic with gravity compensation.
     * ContinuousWrap (configured in configureMotors) handles short-path targeting.
     */
    public void lowerIntakeArm() {
        targetPosition = degreesToMechanismRotations(armLoweredPosition);
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Raise the intake arm using onboard Motion Magic with gravity compensation.
     * ContinuousWrap (configured in configureMotors) handles short-path targeting.
     */
    public void raiseIntakeArm() {
        restoreBrakeMode();
        targetPosition = degreesToMechanismRotations(armRaisedPosition);
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Hold the intake arm at its current position using closed-loop control.
     * Uses Motion Magic to maintain gravity compensation.
     */
    public void stopIntakeArm() {
        restoreBrakeMode();
        targetPosition = intakeArmMotor.getPosition();
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Get the intake roller motor RPM.
     */
    public double getIntakeRPM() {
        return intakeMotor.getVelocity().in(RPM);
    }

    /**
     * Get the intake arm position in degrees.
     */
    public double getIntakeArmPosition() {
        return intakeArmMotor.getPosition() * 360.0;
    }

    /**
     * Convert a throughbore angle (degrees) to mechanism rotations for Motion
     * Magic.
     * With sensorToMechanismRatio configured, all motor position methods operate
     * in mechanism rotations — no gear ratio multiplication needed here.
     */
    private double degreesToMechanismRotations(double degrees) {
        return degrees / 360.0;
    }

    // ==================== Command Factory Methods ====================

    /**
     * Command to run the intake while held.
     */
    public Command runIntakeCommand() {
        return runEnd(this::runIntake, this::stopIntake)
                .withName("Run Intake");
    }

    /**
     * Command to reverse the intake while held.
     */
    public Command reverseIntakeCommand() {
        return runEnd(this::reverseIntake, this::stopIntake)
                .withName("Reverse Intake");
    }

    /**
     * Command to stop the intake (instant).
     */
    public Command stopIntakeCommand() {
        return runOnce(this::stopIntake)
                .withName("Stop Intake");
    }

    /**
     * Command to lower the intake arm using onboard Motion Magic.
     */
    public Command runLowerIntakeArmCommand() {
        return runOnce(this::lowerIntakeArm)
                .withName("Lower Intake Arm");
    }

    /**
     * Command to raise the intake arm using onboard Motion Magic.
     */
    public Command runRaiseIntakeArmCommand() {
        return runOnce(this::raiseIntakeArm)
                .withName("Raise Intake Arm");
    }

    /**
     * Command to stop the intake arm.
     */
    public Command stopIntakeArmCommand() {
        return runOnce(this::stopIntakeArm)
                .withName("Stop Intake Arm");
    }

    /**
     * Restore brake mode if currently in coast (for raising, bopping, or holding).
     */
    private void restoreBrakeMode() {
        if (armInCoast) {
            intakeArmMotor.setBrakeMode(true);
            armInCoast = false;
        }
    }

    /**
     * Shared bop oscillation state machine. Oscillates the arm between two
     * absolute throughbore positions (kBopBottomPosition and kBopTopPosition).
     *
     * <p>Endpoints are clamped to the [lowered, raised] range as a safety
     * rail, so a bad constant can't drive the arm past its physical stops.
     *
     * <p>ContinuousWrap (enabled in configureMotors) guarantees Motion Magic
     * takes the short path to each endpoint even if the multi-turn
     * accumulator differs from the absolute degree values in constants.
     *
     * @param runExtra extra action to run each execute cycle (e.g. run intake
     *                 motor), or null
     */
    private Command bopCommandBase(Runnable runExtra, String name) {
        // Timer-based bop: flip between top and bottom every kBopPhaseSeconds,
        // regardless of whether the arm reached the target. This replaced the
        // earlier position-based design, which stalled when the motor couldn't
        // physically reach kBopTopPosition (saw this at MICMP1 practice — arm
        // maxed out around 140° while target was 150°, atTarget never fired,
        // state machine stuck).
        //
        // Time-based flipping always makes forward progress: if the arm can't
        // reach a target in one phase, we just move on to the other direction
        // after the phase timer expires. Worst case is a partial oscillation
        // instead of a total stall.
        double[] lastFlip = { 0 };
        boolean[] goingUp = { true };
        return run(() -> {
            if (Timer.getFPGATimestamp() - lastFlip[0] >= IntakeConstants.kBopPhaseSeconds) {
                goingUp[0] = !goingUp[0];
                lastFlip[0] = Timer.getFPGATimestamp();
                targetPosition = degreesToMechanismRotations(
                    goingUp[0] ? IntakeConstants.kBopTopPosition
                               : IntakeConstants.kBopBottomPosition);
                intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
            }
            if (runExtra != null) runExtra.run();
        })
        .beforeStarting(() -> {
            // Pick initial direction: if arm is below midpoint, go up first.
            double currentDeg = intakeArmMotor.getPosition() * 360.0;
            double midpoint = (IntakeConstants.kBopBottomPosition
                             + IntakeConstants.kBopTopPosition) / 2.0;
            goingUp[0] = currentDeg < midpoint;
            targetPosition = degreesToMechanismRotations(
                goingUp[0] ? IntakeConstants.kBopTopPosition
                           : IntakeConstants.kBopBottomPosition);
            intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
            lastFlip[0] = Timer.getFPGATimestamp();
            // Mirror raiseIntakeArm's pattern — ensure brake mode in case a
            // prior runIntake left the arm in coast.
            restoreBrakeMode();
        })
        .finallyDo(interrupted -> lowerIntakeArm())
        .withName(name);
    }

    /**
     * Command to bop the intake arm — oscillates slightly up and down from the
     * lowered position to nudge balls toward the hopper while shooting.
     * Hold to keep bopping; releases back to lowered on end.
     */
    public Command bopArmCommand() {
        return bopCommandBase(null, "Bop Arm");
    }

    /**
     * Command to bop the intake arm while running intake rollers.
     * Hold to keep bopping; releases back to lowered on end.
     */
    public Command bopArmAndRunCommand() {
        return bopCommandBase(this::runIntake, "Bop Arm and Run");
    }

    private void setupSimulation() {
        // Kraken X60 as Minion approximation (no DCMotor.getMinion() in WPILib)
        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                SimConstants.kSimGearRatio,
                SingleJointedArmSim.estimateMOI(SimConstants.kArmLengthMeters, SimConstants.kArmMassKg),
                SimConstants.kArmLengthMeters,
                SimConstants.kArmMinAngleRad,
                SimConstants.kArmMaxAngleRad,
                true, // simulate gravity
                SimConstants.kArmStartAngleRad);

        // Mechanism2d canvas — pivot at ~1/3 from left edge
        mech2d = new Mechanism2d(3, 3);
        var armRoot = mech2d.getRoot("ArmPivot", 1.0, 1.0);
        armLigament = armRoot.append(
                new MechanismLigament2d("IntakeArm", 1.0, 0, 6, new Color8Bit(Color.kYellow)));
        SmartDashboard.putData("Intake Arm Sim", mech2d);
    }

    @Override
    public void simulationPeriodic() {
        if (armSim == null)
            return;

        // We compute the motor voltage ourselves instead of reading CTRE's sim output.
        // Why: the motor position convention is inverted (negative = arm up), so
        // negating
        // CTRE's voltage for the PID direction also negates the gravity feedforward,
        // making kG push the arm DOWN instead of holding it up. Additionally, with
        // sensorToMechanismRatio=1.0, CTRE's Arm_Cosine uses cos(motorRotations) which
        // cycles at the wrong rate. Computing voltage from the actual arm angle fixes
        // both.

        double currentAngleRad = armSim.getAngleRads();
        double velocityRadPerSec = armSim.getVelocityRadPerSec();

        // Convert target to arm angle (radians).
        // Motor negative = arm up, so negate. targetPosition is in mechanism rotations.
        double targetAngleRad = -targetPosition * 2 * Math.PI;

        // Error in mechanism rotations (matches CTRE's gain units when scaled by gear
        // ratio)
        double errorMechRot = Units.radiansToRotations(targetAngleRad - currentAngleRad);
        double velocityMechRPS = Units.radiansToRotations(velocityRadPerSec);

        // Compute voltage using tunable gains (gains are in mechanism units)
        double voltage = simP * errorMechRot
                + simKG * Math.cos(currentAngleRad)
                + simKS * Math.signum(errorMechRot)
                - simD * velocityMechRPS;

        // Clamp to battery voltage
        double batteryVoltage = RobotController.getBatteryVoltage();
        simMotorVoltage = Math.max(-batteryVoltage, Math.min(batteryVoltage, voltage));

        // Update physics
        armSim.setInputVoltage(simMotorVoltage);
        armSim.update(0.020);

        // Feed position/velocity back to CTRE for telemetry (getPosition(),
        // getVelocity())
        // Negate: sim positive angle = arm up, motor negative position = arm up
        double mechRotations = Units.radiansToRotations(armSim.getAngleRads());
        double mechRPS = Units.radiansToRotations(armSim.getVelocityRadPerSec());
        intakeArmMotor.setSimSupplyVoltage(batteryVoltage);
        intakeArmMotor.setSimPosition(-mechRotations * IntakeConstants.kArmTotalGearRatio);
        intakeArmMotor.setSimVelocity(-mechRPS * IntakeConstants.kArmTotalGearRatio);

        // Update Mechanism2d visualization
        simArmAngleDeg = Math.toDegrees(armSim.getAngleRads());
        armLigament.setAngle(simArmAngleDeg);
    }

    @Override
    public void periodic() {
        // ESSENTIAL: currents + arm position + roller velocity + jam state.
        // Arm position in degrees lets us verify mechanical integrity post-match.
        // Roller RPS distinguishes "commanded zero" from "stalled" without reading duty.
        log(ESSENTIAL, "Intake/RollerCurrent", intakeMotor.getCurrentDraw().in(Amps));
        log(ESSENTIAL, "Intake/ArmCurrent", intakeArmMotor.getCurrentDraw().in(Amps));
        log(ESSENTIAL, "Intake/ArmPositionDeg", intakeArmMotor.getPosition() * 360.0);
        log(ESSENTIAL, "Intake/RollerRPS", intakeMotor.getVelocity().in(RotationsPerSecond));
        log(ESSENTIAL, "Intake/RollerReversing", rollerReversing);
        // DIAGNOSTIC: Motion-Magic setpoint + velocity lets us see profile tracking
        log(DIAGNOSTIC, "Intake/ArmTargetRotations", targetPosition);
        log(DIAGNOSTIC, "Intake/ArmVelocityRPS", intakeArmMotor.getVelocity().in(RotationsPerSecond));

        // ==================== Roller jam detection ====================
        //
        // State fields:
        // rollerReversing — true while motors run backward to clear a jam
        // wasRollerRunningLastCycle — tracks rising edge (stopped → running) to start
        // grace period
        // rollerJamTimer — reused for both reverse duration and grace period timing
        //
        // Normal sequence:
        // 1. runIntakeCommand() starts → runIntake() sets motor forward each cycle
        // 2. periodic() detects rising edge → restarts timer (grace period begins)
        // 3. After grace period (0.25s): motor has spun up, jam check activates
        // 4. If velocity stays above threshold → no jam, roller runs normally
        // 5. Command ends → stopIntake() sets motor to 0 and clears rollerReversing
        //
        // Jam sequence:
        // 1. Ball jams roller → velocity drops below threshold (0.5 RPS)
        // 2. periodic() detects jam → sets rollerReversing=true, calls reverseIntake()
        // 3. runIntake() sees rollerReversing → skips forward set → reverse continues
        // 4. After reverse duration (0.3s) → rollerReversing=false, timer restarts
        // (grace)
        // 5. Next runIntake() call resumes forward → grace period prevents immediate
        // re-trigger
        // 6. If jam persists → cycle repeats (buzzes back and forth as operator
        // feedback)
        //
        boolean rollerRunning = intakeMotor.getOutputPercent() > 0;

        // --- REVERSING state: roller is running backward to clear a jam ---
        if (rollerReversing) {
            if (rollerJamTimer.hasElapsed(IntakeConstants.kRollerJamReverseDuration)) {
                rollerReversing = false;
                rollerJamTimer.restart(); // reuse as grace timer
            }
            wasRollerRunningLastCycle = false;
            return;
        }

        // --- RUNNING state: detect rising edge to start grace period ---
        if (rollerRunning && !wasRollerRunningLastCycle) {
            rollerJamTimer.restart();
        }

        // --- JAM CHECK: only after grace period has elapsed ---
        if (rollerRunning
                && rollerJamTimer.hasElapsed(IntakeConstants.kRollerJamGracePeriod)
                && intakeMotor.getVelocity().in(RotationsPerSecond) < IntakeConstants.kRollerJamVelocityThreshold) {
            rollerReversing = true;
            rollerJamTimer.restart();
            reverseIntake();
        }

        wasRollerRunningLastCycle = rollerRunning;
    }
}