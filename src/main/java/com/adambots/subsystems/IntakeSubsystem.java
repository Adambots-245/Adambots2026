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

    // Cached PID gains for simulation voltage computation (updated by setArmPID)
    private double simP = IntakeConstants.kArmP;
    private double simD = IntakeConstants.kArmD;
    private double simKG = IntakeConstants.kArmKG;
    private double simKS = IntakeConstants.kArmKS;

    /**
     * Returns true when the intake roller is running (velocity above threshold).
     */
    public Trigger isRunningTrigger() {
        return new Trigger(() -> Math.abs(intakeMotor.getVelocity().in(RotationsPerSecond)) > 0.1);
    }

    // Sign that maps encoder-increasing direction to motor direction.
    // Lowered > Raised → encoder increases as arm lowers → motor needs negative direction.
    private static final int kArmDirection =
        (IntakeConstants.kArmLoweredPosition - IntakeConstants.kArmRaisedPosition) < 0 ? 1 : -1;

    private double targetPosition = 0;

    public IntakeSubsystem(BaseMotor intakeMotor, BaseMotor intakeArmMotor, BaseAbsoluteEncoder armEncoder) {
        this.intakeMotor = intakeMotor;
        this.intakeArmMotor = intakeArmMotor;
        this.armEncoder = armEncoder;

        configureMotors();

        // Seed motor encoder from throughbore absolute position so Motion Magic targets are correct
        double absoluteDeg = armEncoder.getPosition().in(Degrees);
        intakeArmMotor.setPosition(degreesToMechanismRotations(absoluteDeg));
        targetPosition = degreesToMechanismRotations(absoluteDeg);

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
                .currentLimits(IntakeConstants.kRollerStatorCurrentLimit, IntakeConstants.kRollerSupplyCurrentLimit, 3500)
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
        Dash.add("Arm Encoder (deg)", () -> armEncoder.getPosition().in(Degrees), 3, 0);
        Dash.add("Arm Mech Pos (rot)", () -> intakeArmMotor.getPosition(), 4, 0);
        Dash.add("Arm Target (mech rot)", () -> targetPosition, 5, 0);
        Dash.add("Arm Direction", () -> kArmDirection, 8, 0);
        Dash.add("Target (deg)", () -> targetPosition / kArmDirection * 360.0, 9, 0);

        // Row 0 (cont.): Sim diagnostics (only meaningful in simulation)
        Dash.add("Sim Voltage", () -> simMotorVoltage, 6, 0);
        Dash.add("Sim Angle Deg", () -> simArmAngleDeg, 7, 0);

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

    // ==================== Tuning Setters (called by TuningManager) ====================

    public void setArmPID(double p, double i, double d, double kG, double kS, double kV, double kA) {
        intakeArmMotor.setPID(0, p, i, d, kV, kS, kA, kG);
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
        simP = p; simD = d; simKG = kG; simKS = kS;
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
        intakeMotor.set(IntakeConstants.kIntakeSpeed);
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
        targetPosition = degreesToMechanismRotations(IntakeConstants.kArmRaisedPosition);
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Hold the intake arm at its current position using closed-loop control.
     * Uses Motion Magic to maintain gravity compensation.
     */
    public void stopIntakeArm() {
        targetPosition = degreesToMechanismRotations(armEncoder.getPosition().in(Degrees));
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Get the intake roller motor RPM.
     */
    public double getIntakeRPM() {
        return intakeMotor.getVelocity().in(RPM);
    }

    /**
     * Get the intake arm position in degrees from the throughbore encoder.
     */
    public double getIntakeArmPosition() {
        return armEncoder.getPosition().in(Degrees);
    }

    /**
     * Convert a throughbore angle (degrees) to mechanism rotations for Motion Magic.
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

    /**
     * Shared bop oscillation state machine. Oscillates the arm between lowered
     * and (lowered - bopAngle) positions. Returns a runEnd command base.
     *
     * @param runExtra extra action to run each execute cycle (e.g. run intake motor), or null
     */
    private Command bopCommandBase(Runnable runExtra, String name) {
        boolean[] bopUp = {false};
        double[] switchTime = {0};
        return runEnd(
            () -> {
                double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                if (switchTime[0] == 0) switchTime[0] = now;
                double lowered = degreesToMechanismRotations(armLoweredPosition);
                double raised = degreesToMechanismRotations(armLoweredPosition - bopAngle);
                if (now - switchTime[0] > 0.35) {
                    bopUp[0] = !bopUp[0];
                    switchTime[0] = now;
                }
                targetPosition = bopUp[0] ? raised : lowered;
                intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
                if (runExtra != null) runExtra.run();
            },
            () -> {
                bopUp[0] = false;
                lowerIntakeArm();
            }
        ).withName(name);
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
        return bopCommandBase(() -> intakeMotor.set(IntakeConstants.kIntakeSpeed), "Bop Arm and Run");
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
     * Check if the arm's throughbore encoder is within tolerance of the current target.
     */
    private boolean isArmAtTarget() {
        double currentDeg = armEncoder.getPosition().in(Degrees);
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
        double tolerance = 0.01; // mechanism rotations
        return Math.abs(targetPosition - raisedTarget) < tolerance
            || Math.abs(targetPosition - loweredTarget) < tolerance;
    }

    @Override
    public void periodic() {
        // Re-sync motor encoder from throughbore when arm has settled at a known setpoint.
        // Only re-sync when velocity is near zero and at raised/lowered position,
        // not during bop oscillation where it could cause position jumps.
        if (isArmAtTarget() && isAtKnownSetpoint()
                && Math.abs(intakeArmMotor.getVelocity().in(RotationsPerSecond)) < 0.05) {
            double absoluteDeg = armEncoder.getPosition().in(Degrees);
            intakeArmMotor.setPosition(degreesToMechanismRotations(absoluteDeg));
        }
    }
}