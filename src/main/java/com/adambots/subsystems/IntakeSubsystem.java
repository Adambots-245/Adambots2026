package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.adambots.Constants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.SimConstants;
import com.adambots.Robot;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.utils.Dash;

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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Intake subsystem using TalonFX onboard PID with gravity compensation.
 *
 * <p>
 * The arm motor runs Motion Magic with Arm_Cosine gravity feedforward at 1kHz
 * on the motor controller, providing stable holding and smooth motion.
 */
@Logged
public class IntakeSubsystem extends SubsystemBase {

    private final BaseMotor intakeMotor;
    private final BaseMotor intakeArmMotor;

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

    private double targetPosition = IntakeConstants.kArmLoweredPosition;

    public IntakeSubsystem(BaseMotor intakeMotor, BaseMotor intakeArmMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeArmMotor = intakeArmMotor;

        configureMotors();
        if (Constants.INTAKE_TAB) {
            setupDash();
        }
        // intakeArmMotor.setPosition(0);

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
                .currentLimits(IntakeConstants.kArmStatorCurrentLimit, IntakeConstants.kArmSupplyCurrentLimit, 2500)
                .gravity(BaseMotor.GravityType.ARM_COSINE)
                .sensorToMechanismRatio(IntakeConstants.kArmTotalGearRatio)
                .motionMagic(
                        RotationsPerSecond.of(IntakeConstants.kArmCruiseVelocity),
                        RotationsPerSecondPerSecond.of(IntakeConstants.kArmAcceleration),
                        IntakeConstants.kArmJerk)
                .apply();
        
        intakeArmMotor.configureHardLimits(false, true, 0, 0);

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
        Dash.add("Arm Position", () -> intakeArmMotor.getPosition(), 3, 0);
        Dash.add("Arm Target", () -> targetPosition, 4, 0);
        Dash.add("Raised Position", () -> IntakeConstants.kArmRaisedPosition, 5, 0);

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
        Dash.addCommand("Reset Position", resetIntakeArmPosition(), 6, 1);
        Dash.addCommand("Bop Arm", bopArmCommand(), 7, 1);

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
        intakeMotor.set(IntakeConstants.kLowSpeed);
    }

    /**
     * Run the intake roller in reverse.
     */
    public void reverseIntake() {
        intakeMotor.set(-IntakeConstants.kLowSpeed);
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
        targetPosition = armLoweredPosition;
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Raise the intake arm using onboard Motion Magic with gravity compensation.
     */
    public void raiseIntakeArm() {
        targetPosition = IntakeConstants.kArmRaisedPosition;
        intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
    }

    /**
     * Hold the intake arm at its current position using closed-loop control.
     * Uses Motion Magic to maintain gravity compensation.
     */
    public void stopIntakeArm() {
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
     * Get the intake arm motor position.
     */
    public double getIntakeArmPosition() {
        return intakeArmMotor.getPosition();
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
     * Reset arm position encoder to 0.
     */
    public Command resetIntakeArmPosition() {
        return runOnce(() -> intakeArmMotor.setPosition(0))
                .withName("Reset Intake Position");
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
     * Command to bop the intake arm — oscillates slightly up and down from the
     * lowered position to nudge balls toward the hopper while shooting.
     * Hold to keep bopping; releases back to lowered on end.
     */
    public Command bopArmCommand() {
        // Track which phase we're in (up vs down)
        boolean[] bopUp = {true};
        double[] switchTime = {0};
        return runEnd(
            () -> {
                double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                double lowered = armLoweredPosition;
                double raised = lowered - bopAngle;  // negative = up
                // Switch direction based on elapsed time since last switch
                if (now - switchTime[0] > 0.35) {
                    bopUp[0] = !bopUp[0];
                    switchTime[0] = now;
                }
                targetPosition = bopUp[0] ? raised : lowered;
                intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
            },
            () -> {
                bopUp[0] = true;
                lowerIntakeArm();
            }
        ).withName("Bop Arm");
    }

        /**
     * Command to bop the intake arm — oscillates slightly up and down from the
     * lowered position to nudge balls toward the hopper while shooting.
     * Hold to keep bopping; releases back to lowered on end.
     */
    public Command bopArmAndRunCommand() {
        // Track which phase we're in (up vs down)
        boolean[] bopUp = {true};
        double[] switchTime = {0};
        return runEnd(
            () -> {
                double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                double lowered = armLoweredPosition;
                double raised = lowered - bopAngle;  // negative = up
                // Switch direction based on elapsed time since last switch
                if (now - switchTime[0] > 0.35) {
                    bopUp[0] = !bopUp[0];
                    switchTime[0] = now;
                }
                targetPosition = bopUp[0] ? raised : lowered;
                intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
                // TODO(vx-clutch): factor out the duplicated logic from the regular bop and just add this
                intakeMotor.set(IntakeConstants.kHighSpeed);
            },
            () -> {
                bopUp[0] = true;
                lowerIntakeArm();
            }
        ).withName("Bop Arm and Run");
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

        // Convert target from motor rotations to arm angle (radians)
        // Motor negative = arm up, so negate. Divide by gear ratio to get mechanism
        // rotations.
        double targetAngleRad = -targetPosition / SimConstants.kSimGearRatio * 2 * Math.PI;

        // Error in mechanism rotations (matches CTRE's gain units when scaled by gear
        // ratio)
        double errorMechRot = Units.radiansToRotations(targetAngleRad - currentAngleRad);
        double velocityMechRPS = Units.radiansToRotations(velocityRadPerSec);

        // Compute voltage using tunable gains
        // Scale position/velocity terms by gear ratio so CTRE gain values produce
        // equivalent torque (CTRE PID operates in motor rotations = mech * gearRatio)
        double voltage = simP * errorMechRot * SimConstants.kSimGearRatio
                + simKG * Math.cos(currentAngleRad)
                + simKS * Math.signum(errorMechRot)
                - simD * velocityMechRPS * SimConstants.kSimGearRatio;

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
        intakeArmMotor.setSimPosition(-mechRotations * SimConstants.kSimGearRatio);
        intakeArmMotor.setSimVelocity(-mechRPS * SimConstants.kSimGearRatio);

        // Update Mechanism2d visualization
        simArmAngleDeg = Math.toDegrees(armSim.getAngleRads());
        armLigament.setAngle(simArmAngleDeg);
    }

    @Override
    public void periodic() {
        // When the reverse limit switch is active and we're targeting the raised position,
        // snap the target to 0 so the motor stops pushing against the hard stop.
        // The hardware limit already auto-reset the encoder to 0.
        if (intakeArmMotor.getReverseLimitSwitch() && targetPosition < 0) {
            targetPosition = 0;
            intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, 0);
        }

    }
}