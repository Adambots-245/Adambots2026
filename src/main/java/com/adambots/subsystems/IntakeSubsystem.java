package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.SimConstants;
import com.adambots.Robot;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.sensors.BaseAbsoluteEncoder;
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

import org.littletonrobotics.junction.Logger;

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
    private final BaseAbsoluteEncoder armEncoder;

    // Simulation
    private SingleJointedArmSim armSim;
    private Mechanism2d mech2d;
    private MechanismLigament2d armLigament;
    private double simMotorVoltage;
    private double simArmAngleDeg;

    private double bopAngle = IntakeConstants.kBopAngle;
    private double armLoweredPosition = IntakeConstants.kArmLoweredPosition;

    // Coast-when-lowered: arm goes loose at lowered position for compliance
    private boolean armInCoast = false;

    // Roller jam detection state
    private boolean rollerReversing = false;
    private boolean wasRollerRunningLastCycle = false;
    private final edu.wpi.first.wpilibj.Timer rollerJamTimer = new edu.wpi.first.wpilibj.Timer();

    // Cached PID gains for simulation voltage computation (updated by setArmPID)
    private double simP = IntakeConstants.kArmP;
    private double simD = IntakeConstants.kArmD;
    private double simKG = IntakeConstants.kArmKG;
    private double simKS = IntakeConstants.kArmKS;

    /**
     * Returns true when the intake roller is running (velocity above threshold).
     */
    public Trigger isRunningTrigger() {
        return new Trigger(() -> Math.abs(intakeMotor.getVelocity().in(RotationsPerSecond)) > IntakeConstants.kRollerRunningThreshold);
    }

    // Sign that maps encoder-increasing direction to motor direction.
    // Lowered > Raised → encoder increases as arm lowers → motor needs negative
    // direction.
    private static final int kArmDirection = (IntakeConstants.kArmLoweredPosition
            - IntakeConstants.kArmRaisedPosition) < 0 ? 1 : -1;

    private double targetPosition = 0;

    public IntakeSubsystem(BaseMotor intakeMotor, BaseMotor intakeArmMotor, BaseAbsoluteEncoder armEncoder) {
        this.intakeMotor = intakeMotor;
        this.intakeArmMotor = intakeArmMotor;
        this.armEncoder = armEncoder;

        configureMotors();

        // Seed motor encoder from throughbore absolute position so Motion Magic targets
        // are correct. Skip when using external encoder — FXS reads throughbore directly.
        if (!IntakeConstants.kUseExternalEncoder) {
            double absoluteDeg = armEncoder.getPosition().in(Degrees);
            intakeArmMotor.setPosition(degreesToMechanismRotations(absoluteDeg));
            targetPosition = degreesToMechanismRotations(absoluteDeg);
        }

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

        // When throughbore is hardwired to TalonFXS data port, use it as the feedback source.
        // FXS reads PWM signal directly at 1kHz — eliminates DIO polling and re-sync.
        // Ratio = 1.0 because throughbore is on the mechanism side (post-gearbox).
        if (IntakeConstants.kUseExternalEncoder) {
            intakeArmMotor.configureExternalPulseWidthSensor(1.0, 0.0, 1.0);
        }

        // Set extended PID with feedforward gains (kV, kS, kA, kG)
        intakeArmMotor.setPID(0,
                IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD,
                IntakeConstants.kArmKV, IntakeConstants.kArmKS, IntakeConstants.kArmKA,
                IntakeConstants.kArmKG);

        // Re-apply gravity type AFTER setPID — setPID overwrites Slot0Configs
        // which can reset GravityType to the default (Elevator_Static)
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
    }

    private void setupDash() {
        Dash.useTab("Intake");

        // Row 0: Telemetry
        Dash.add("Roller Speed", () -> intakeMotor.getVelocity().in(RotationsPerSecond), 0, 0);
        Dash.add("Roller Position", () -> intakeMotor.getPosition(), 1, 0);
        Dash.add("Arm Speed", () -> intakeArmMotor.getVelocity().in(RotationsPerSecond), 2, 0);
        // DIO throughbore when using roboRIO; FXS position converted to degrees when external
        Dash.add("Arm Encoder (deg)", () -> IntakeConstants.kUseExternalEncoder
            ? intakeArmMotor.getPosition() * 360.0
            : armEncoder.getPosition().in(Degrees), 3, 0);
        Dash.add("Arm Mech Pos (rot)", () -> intakeArmMotor.getPosition(), 4, 0);
        Dash.add("Arm Target (mech rot)", () -> targetPosition, 5, 0);
        Dash.add("Arm At Target", () -> isArmAtTarget(), 6, 0);
        Dash.add("Arm Direction", () -> kArmDirection, 7, 0);
        Dash.add("Target (deg)", () -> targetPosition / kArmDirection * 360.0, 8, 0);
        Dash.add("Ext Encoder", () -> IntakeConstants.kUseExternalEncoder, 9, 0);
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

    public void setBopAngle(double angle) {
        bopAngle = angle;
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
     */
    public void lowerIntakeArm() {
        targetPosition = degreesToMechanismRotations(armLoweredPosition);
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Raise the intake arm using onboard Motion Magic with gravity compensation.
     */
    public void raiseIntakeArm() {
        restoreBrakeMode();
        targetPosition = degreesToMechanismRotations(IntakeConstants.kArmRaisedPosition);
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Hold the intake arm at its current position using closed-loop control.
     * Uses Motion Magic to maintain gravity compensation.
     */
    public void stopIntakeArm() {
        restoreBrakeMode();
        targetPosition = IntakeConstants.kUseExternalEncoder
            ? intakeArmMotor.getPosition()
            : degreesToMechanismRotations(armEncoder.getPosition().in(Degrees));
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
        return IntakeConstants.kUseExternalEncoder
            ? intakeArmMotor.getPosition() * 360.0
            : armEncoder.getPosition().in(Degrees);
    }

    /**
     * Convert a throughbore angle (degrees) to mechanism rotations for Motion
     * Magic.
     * With sensorToMechanismRatio configured, all motor position methods operate
     * in mechanism rotations — no gear ratio multiplication needed here.
     */
    private double degreesToMechanismRotations(double degrees) {
        return kArmDirection * (degrees / 360.0);
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

    /** Restore brake mode if currently in coast (for raising, bopping, or holding). */
    private void restoreBrakeMode() {
        if (armInCoast) {
            intakeArmMotor.setBrakeMode(true);
            armInCoast = false;
        }
    }

    /**
     * Shared bop oscillation state machine. Oscillates the arm between lowered
     * and (lowered - bopAngle) positions. Returns a runEnd command base.
     *
     * @param runExtra extra action to run each execute cycle (e.g. run intake
     *                 motor), or null
     */
    private Command bopCommandBase(Runnable runExtra, String name) {
        boolean[] bopUp = { false };
        double[] switchTime = { 0 };
        return runEnd(
                () -> {
                    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                    if (switchTime[0] == 0)
                        switchTime[0] = now;
                    double lowered = degreesToMechanismRotations(armLoweredPosition);
                    double raised = degreesToMechanismRotations(armLoweredPosition - bopAngle);
                    if (now - switchTime[0] > IntakeConstants.kBopSwitchTimeSeconds) {
                        bopUp[0] = !bopUp[0];
                        switchTime[0] = now;
                    }
                    targetPosition = bopUp[0] ? raised : lowered;
                    intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
                    if (runExtra != null)
                        runExtra.run();
                },
                () -> {
                    bopUp[0] = false;
                    lowerIntakeArm();
                }).withName(name);
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

    /**
     * Check if the arm's throughbore encoder is within tolerance of the current
     * target.
     */
    private boolean isArmAtTarget() {
        double currentDeg = getIntakeArmPosition();
        double targetDeg = targetPosition / kArmDirection * 360.0;
        return Math.abs(currentDeg - targetDeg) < IntakeConstants.kArmAtTargetThreshold;
    }

    /**
     * Check if the target is one of the two known setpoints (raised or lowered).
     * Used to guard re-sync so it doesn't fire during bop oscillation.
     */
    private boolean isAtKnownSetpoint() {
        double raisedTarget = degreesToMechanismRotations(IntakeConstants.kArmRaisedPosition);
        double loweredTarget = degreesToMechanismRotations(armLoweredPosition);
        double tolerance = IntakeConstants.kArmKnownSetpointTolerance;
        return Math.abs(targetPosition - raisedTarget) < tolerance
                || Math.abs(targetPosition - loweredTarget) < tolerance;
    }

    @Override
    public void periodic() {
        if (Constants.CURRENT_LOGGING) {
            Logger.recordOutput("Intake/RollerCurrent", intakeMotor.getCurrentDraw().in(Amps));
            Logger.recordOutput("Intake/ArmCurrent", intakeArmMotor.getCurrentDraw().in(Amps));
        }

        // ==================== Arm encoder re-sync ====================
        // Re-sync motor encoder from throughbore when arm has settled at a known
        // setpoint. Only re-syncs when: (1) arm is at target, (2) target is
        // raised or lowered (not mid-bop), (3) velocity is near zero.
        // Prevents position jumps during motion.
        // Skip when using external encoder — FXS reads throughbore directly, no drift.
        if (!IntakeConstants.kUseExternalEncoder
                && isArmAtTarget() && isAtKnownSetpoint()
                && Math.abs(intakeArmMotor.getVelocity().in(RotationsPerSecond)) < 0.05) {
            double absoluteDeg = armEncoder.getPosition().in(Degrees);
            intakeArmMotor.setPosition(degreesToMechanismRotations(absoluteDeg));
        }

        // ==================== Roller jam detection ====================
        //
        // State fields:
        //   rollerReversing        — true while motors run backward to clear a jam
        //   wasRollerRunningLastCycle — tracks rising edge (stopped → running) to start grace period
        //   rollerJamTimer         — reused for both reverse duration and grace period timing
        //
        // Normal sequence:
        //   1. runIntakeCommand() starts → runIntake() sets motor forward each cycle
        //   2. periodic() detects rising edge → restarts timer (grace period begins)
        //   3. After grace period (0.25s): motor has spun up, jam check activates
        //   4. If velocity stays above threshold → no jam, roller runs normally
        //   5. Command ends → stopIntake() sets motor to 0 and clears rollerReversing
        //
        // Jam sequence:
        //   1. Ball jams roller → velocity drops below threshold (0.5 RPS)
        //   2. periodic() detects jam → sets rollerReversing=true, calls reverseIntake()
        //   3. runIntake() sees rollerReversing → skips forward set → reverse continues
        //   4. After reverse duration (0.3s) → rollerReversing=false, timer restarts (grace)
        //   5. Next runIntake() call resumes forward → grace period prevents immediate re-trigger
        //   6. If jam persists → cycle repeats (buzzes back and forth as operator feedback)
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