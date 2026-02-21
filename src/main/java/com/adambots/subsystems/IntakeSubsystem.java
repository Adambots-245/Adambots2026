package com.adambots.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.SimConstants;
import com.adambots.Robot;
import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final DigitalInput limitSwitch;

    // Simulation
    private SingleJointedArmSim armSim;
    private Mechanism2d mech2d;
    private MechanismLigament2d armLigament;
    private double simMotorVoltage;
    private double simArmAngleDeg;

    // Tunable PID entries
    private GenericEntry intakeArmPEntry;
    private GenericEntry intakeArmIEntry;
    private GenericEntry intakeArmDEntry;
    private GenericEntry intakeArmKGEntry;
    private GenericEntry intakeArmKSEntry;
    private GenericEntry intakeArmKVEntry;
    private GenericEntry intakeArmKAEntry;

    /**
     * Returns true when a game piece is detected in the intake.
     */
    public Trigger hasGamePieceTrigger() {
        // TODO: Implement sensor logic - check if sensor detects object
        // return new Trigger(() -> intakeSensor.isDetected());
        return new Trigger(() -> false);
    }

    /**
     * Returns true when the intake is running.
     */
    public Trigger isRunningTrigger() {
        // TODO: Check if motor is running above threshold
        // return new Trigger(() -> Math.abs(intakeMotor.getVelocity()) > 0.1);
        return new Trigger(() -> false);
    }

    private double lastP, lastI, lastD, lastKG, lastKS, lastKV, lastKA;

    private double targetPosition = IntakeConstants.kArmLoweredPosition;

    public IntakeSubsystem(BaseMotor intakeMotor, BaseMotor intakeArmMotor, DigitalInput limitSwitch) {
        this.intakeMotor = intakeMotor;
        this.intakeArmMotor = intakeArmMotor;
        this.limitSwitch = limitSwitch;

        configureMotors();
        setupDash();
        configureLimitSwitch();
        intakeArmMotor.setPosition(0);

        if (Robot.isSimulation()) {
            setupSimulation();
        }
    }

    private void configureMotors() {
        intakeMotor.setBrakeMode(false);

        // Configure arm motor: brake mode + current limits + gear ratio
        intakeArmMotor.configure()
                .brakeMode(true)
                .currentLimits(IntakeConstants.kArmStatorCurrentLimit, IntakeConstants.kArmSupplyCurrentLimit, 0)
                .gravity(BaseMotor.GravityType.ARM_COSINE)
                .sensorToMechanismRatio(IntakeConstants.kArmTotalGearRatio)
                .motionMagic(
                        RotationsPerSecond.of(IntakeConstants.kArmCruiseVelocity),
                        RotationsPerSecondPerSecond.of(IntakeConstants.kArmAcceleration),
                        IntakeConstants.kArmJerk)
                .apply();

        // Set extended PID with feedforward gains (kV, kS, kA, kG)
        intakeArmMotor.setPID(0,
                IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD,
                IntakeConstants.kArmKV, IntakeConstants.kArmKS, IntakeConstants.kArmKA,
                IntakeConstants.kArmKG);

        // Re-apply gravity type AFTER setPID — setPID overwrites Slot0Configs
        // which can reset GravityType to the default (Elevator_Static)
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
    }

    private void configureLimitSwitch() {
        // DIO returns false when switch is closed/triggered
        new Trigger(() -> !limitSwitch.get())
                .onTrue(Commands.runOnce(() -> {
                    intakeArmMotor.setPosition(IntakeConstants.kArmRaisedPosition);
                    targetPosition = IntakeConstants.kArmRaisedPosition;
                    intakeArmMotor.set(BaseMotor.ControlMode.MOTION_MAGIC, targetPosition);
                }, this).ignoringDisable(true));
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
        Dash.add("Limit Switch", () -> limitSwitch.get(), 6, 0);

        // Row 0 (cont.): Sim diagnostics (only meaningful in simulation)
        Dash.add("Sim Voltage", () -> simMotorVoltage, 7, 0);
        Dash.add("Sim Angle Deg", () -> simArmAngleDeg, 8, 0);

        // Row 1: Commands
        Dash.addCommand("Start Intake", runIntakeCommand(), 0, 1);
        Dash.addCommand("Reverse Intake", reverseIntakeCommand(), 1, 1);
        Dash.addCommand("Stop Intake", stopIntakeCommand(), 2, 1);
        Dash.addCommand("Lower Arm", runLowerIntakeArmCommand(), 3, 1);
        Dash.addCommand("Raise Arm", runRaiseIntakeArmCommand(), 4, 1);
        Dash.addCommand("Stop Arm", stopIntakeArmCommand(), 5, 1);
        Dash.addCommand("Reset Position", resetIntakeArmPosition(), 6, 1);

        Dash.useDefaultTab();
    }

    public void setupTunables() {
        Dash.useTab("Intake");

        // Row 2: Tunable PID / feedforward gains
        intakeArmPEntry = Dash.addTunable("kP", IntakeConstants.kArmP, 0, 2);
        intakeArmIEntry = Dash.addTunable("kI", IntakeConstants.kArmI, 1, 2);
        intakeArmDEntry = Dash.addTunable("kD", IntakeConstants.kArmD, 2, 2);
        intakeArmKGEntry = Dash.addTunable("kG", IntakeConstants.kArmKG, 3, 2);
        intakeArmKSEntry = Dash.addTunable("kS", IntakeConstants.kArmKS, 4, 2);
        intakeArmKVEntry = Dash.addTunable("kV", IntakeConstants.kArmKV, 5, 2);
        intakeArmKAEntry = Dash.addTunable("kA", IntakeConstants.kArmKA, 6, 2);
        Dash.addCommand("Zero Tunables", zeroTunablesCommand(), 7, 2);
        Dash.addCommand("Reset Tunables", resetTunablesCommand(), 8, 2);

        Dash.useDefaultTab();

        // Force-write code constants to Shuffleboard so displayed values always
        // match what's in use (overrides stale cache from previous sessions)
        intakeArmPEntry.setDouble(IntakeConstants.kArmP);
        intakeArmIEntry.setDouble(IntakeConstants.kArmI);
        intakeArmDEntry.setDouble(IntakeConstants.kArmD);
        intakeArmKGEntry.setDouble(IntakeConstants.kArmKG);
        intakeArmKSEntry.setDouble(IntakeConstants.kArmKS);
        intakeArmKVEntry.setDouble(IntakeConstants.kArmKV);
        intakeArmKAEntry.setDouble(IntakeConstants.kArmKA);

        lastP = IntakeConstants.kArmP;
        lastI = IntakeConstants.kArmI;
        lastD = IntakeConstants.kArmD;
        lastKG = IntakeConstants.kArmKG;
        lastKS = IntakeConstants.kArmKS;
        lastKV = IntakeConstants.kArmKV;
        lastKA = IntakeConstants.kArmKA;

        // Apply to motor controller (for real hardware)
        intakeArmMotor.setPID(0,
                IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD,
                IntakeConstants.kArmKV, IntakeConstants.kArmKS, IntakeConstants.kArmKA,
                IntakeConstants.kArmKG);
        intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);
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
        targetPosition = IntakeConstants.kArmLoweredPosition;
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
    public double getintakeRPM() {
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
     * Command to reset all tunable PID/feedforward gains back to code constants.
     */
    public Command resetTunablesCommand() {
        return runOnce(() -> {
            intakeArmPEntry.setDouble(IntakeConstants.kArmP);
            intakeArmIEntry.setDouble(IntakeConstants.kArmI);
            intakeArmDEntry.setDouble(IntakeConstants.kArmD);
            intakeArmKGEntry.setDouble(IntakeConstants.kArmKG);
            intakeArmKSEntry.setDouble(IntakeConstants.kArmKS);
            intakeArmKVEntry.setDouble(IntakeConstants.kArmKV);
            intakeArmKAEntry.setDouble(IntakeConstants.kArmKA);

            intakeArmMotor.setPID(0,
                    IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD,
                    IntakeConstants.kArmKV, IntakeConstants.kArmKS, IntakeConstants.kArmKA,
                    IntakeConstants.kArmKG);
            intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);

            lastP = IntakeConstants.kArmP;
            lastI = IntakeConstants.kArmI;
            lastD = IntakeConstants.kArmD;
            lastKG = IntakeConstants.kArmKG;
            lastKS = IntakeConstants.kArmKS;
            lastKV = IntakeConstants.kArmKV;
            lastKA = IntakeConstants.kArmKA;
        }).withName("Reset Tunables");
    }

    /**
     * Command to zero all tunable PID/feedforward gains (for tuning from scratch).
     */
    public Command zeroTunablesCommand() {
        return runOnce(() -> {
            intakeArmPEntry.setDouble(0);
            intakeArmIEntry.setDouble(0);
            intakeArmDEntry.setDouble(0);
            intakeArmKGEntry.setDouble(0);
            intakeArmKSEntry.setDouble(0);
            intakeArmKVEntry.setDouble(0);
            intakeArmKAEntry.setDouble(0);

            intakeArmMotor.setPID(0, 0, 0, 0, 0, 0, 0, 0);
            intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);

            lastP = 0;
            lastI = 0;
            lastD = 0;
            lastKG = 0;
            lastKS = 0;
            lastKV = 0;
            lastKA = 0;
        }).withName("Zero Tunables");
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
        double voltage = lastP * errorMechRot * SimConstants.kSimGearRatio
                + lastKG * Math.cos(currentAngleRad)
                + lastKS * Math.signum(errorMechRot)
                - lastD * velocityMechRPS * SimConstants.kSimGearRatio;

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
        // No PID loop here - the motor controller handles everything at 1kHz.
        // We only check for tunable gain updates from Shuffleboard.

        if (intakeArmPEntry != null) {
            double p = intakeArmPEntry.getDouble(IntakeConstants.kArmP);
            double i = intakeArmIEntry.getDouble(IntakeConstants.kArmI);
            double d = intakeArmDEntry.getDouble(IntakeConstants.kArmD);
            double kG = intakeArmKGEntry.getDouble(IntakeConstants.kArmKG);
            double kS = intakeArmKSEntry.getDouble(IntakeConstants.kArmKS);
            double kV = intakeArmKVEntry.getDouble(IntakeConstants.kArmKV);
            double kA = intakeArmKAEntry.getDouble(IntakeConstants.kArmKA);
            System.out.println("Current kG:" + kG);

            if (p != lastP || i != lastI || d != lastD ||
                    kG != lastKG || kS != lastKS || kV != lastKV || kA != lastKA) {

                intakeArmMotor.setPID(0, p, i, d, kV, kS, kA, kG);
                System.out.println("Setting PID");
                intakeArmMotor.configureGravity(BaseMotor.GravityType.ARM_COSINE);

                lastP = p;
                lastI = i;
                lastD = d;
                lastKG = kG;
                lastKS = kS;
                lastKV = kV;
                lastKA = kA;
            }
        }
    }
}