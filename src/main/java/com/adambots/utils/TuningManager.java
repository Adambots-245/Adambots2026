package com.adambots.utils;

import com.adambots.Constants;
import com.adambots.Constants.HopperConstants;
import com.adambots.Constants.IntakeConstants;
import com.adambots.Constants.ShooterConstants;
import com.adambots.Constants.TurretConstants;
import com.adambots.Constants.TurretTrackingConstants;
import com.adambots.Constants.VisionConstants;
import com.adambots.lib.utils.Dash;
import com.adambots.subsystems.HopperSubsystem;
import com.adambots.subsystems.IntakeSubsystem;
import com.adambots.subsystems.ShooterSubsystem;
import com.adambots.subsystems.TurretSubsystem;
import com.adambots.subsystems.VisionSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Centralizes all tuning (GenericEntry) reads, change detection, and subsystem setter calls.
 * Created by DashboardSetup when TUNING_ENABLED is true. Called each cycle from Robot.robotPeriodic().
 */
public class TuningManager {

    // Subsystem references
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IntakeSubsystem intake;
    private final HopperSubsystem hopper;
    private final VisionSubsystem vision; // nullable

    // ==================== Shooter entries + cache ====================
    private GenericEntry flywheelPEntry, flywheelIEntry, flywheelDEntry, flywheelFFEntry;
    private GenericEntry flywheelToleranceEntry;
    private final GenericEntry[] tableDistanceEntries = new GenericEntry[5];
    private final GenericEntry[] tableRPSEntries = new GenericEntry[5];
    private GenericEntry lobShotRPSEntry;

    private double lastFlywheelP = ShooterConstants.kFlywheelP;
    private double lastFlywheelI = ShooterConstants.kFlywheelI;
    private double lastFlywheelD = ShooterConstants.kFlywheelD;
    private double lastFlywheelFF = ShooterConstants.kFlywheelFF;
    private final double[] lastTableDist = new double[5];
    private final double[] lastTableRPS = new double[5];

    // ==================== Turret entries + cache ====================
    private GenericEntry turretPEntry, turretIEntry, turretDEntry, turretFFEntry;
    private GenericEntry trackingToleranceEntry;
    private GenericEntry potAtZeroEntry, potAtMaxEntry;

    private double lastTurretP = TurretConstants.kTurretP;
    private double lastTurretI = TurretConstants.kTurretI;
    private double lastTurretD = TurretConstants.kTurretD;
    private double lastTurretFF = TurretConstants.kTurretFF;

    // ==================== Intake entries + cache ====================
    private GenericEntry intakeArmPEntry, intakeArmIEntry, intakeArmDEntry;
    private GenericEntry intakeArmKGEntry, intakeArmKSEntry, intakeArmKVEntry, intakeArmKAEntry;
    private GenericEntry cruiseVelocityEntry, accelerationEntry;
    private GenericEntry loweredPositionEntry, raisedPositionEntry;

    private double lastIntakeP = IntakeConstants.kArmP;
    private double lastIntakeI = IntakeConstants.kArmI;
    private double lastIntakeD = IntakeConstants.kArmD;
    private double lastIntakeKG = IntakeConstants.kArmKG;
    private double lastIntakeKS = IntakeConstants.kArmKS;
    private double lastIntakeKV = IntakeConstants.kArmKV;
    private double lastIntakeKA = IntakeConstants.kArmKA;
    private double lastCruiseVelocity = IntakeConstants.kArmCruiseVelocity;
    private double lastAcceleration = IntakeConstants.kArmAcceleration;

    // ==================== Hopper entries ====================
    private GenericEntry hopperSpeedEntry, uptakeSpeedEntry;

    // ==================== Vision entries ====================
    private GenericEntry visionModeEntry;
    private GenericEntry ambiguityEntry;

    public TuningManager(ShooterSubsystem shooter, TurretSubsystem turret,
                         IntakeSubsystem intake, HopperSubsystem hopper,
                         VisionSubsystem vision) {
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
        this.hopper = hopper;
        this.vision = vision;

        // Initialize interpolation table cache from defaults
        for (int i = 0; i < 5; i++) {
            lastTableDist[i] = ShooterConstants.kDefaultInterpolationTable[i][0];
            lastTableRPS[i] = ShooterConstants.kDefaultInterpolationTable[i][1];
        }
    }

    // ==================== Entry Setup Methods ====================

    /** Creates shooter + turret tunable entries on the Shooter tab. */
    public void setupShooterTunables(int[] pos, int cols) {
        flywheelPEntry = Dash.addTunable("Flywheel kP", ShooterConstants.kFlywheelP, pos[0], pos[1]);
        advance(pos, cols);
        flywheelIEntry = Dash.addTunable("Flywheel kI", ShooterConstants.kFlywheelI, pos[0], pos[1]);
        advance(pos, cols);
        flywheelDEntry = Dash.addTunable("Flywheel kD", ShooterConstants.kFlywheelD, pos[0], pos[1]);
        advance(pos, cols);
        flywheelFFEntry = Dash.addTunable("Flywheel kF", ShooterConstants.kFlywheelFF, pos[0], pos[1]);
        advance(pos, cols);
        flywheelToleranceEntry = Dash.addTunable("Flywheel Tolerance (RPS)", ShooterConstants.kFlywheelToleranceRPS, pos[0], pos[1]);
        advance(pos, cols);

        newRow(pos);
        for (int i = 0; i < 5; i++) {
            tableDistanceEntries[i] = Dash.addTunable(
                "Dist " + (i + 1), ShooterConstants.kDefaultInterpolationTable[i][0], pos[0], pos[1]);
            advance(pos, cols);
        }
        for (int i = 0; i < 5; i++) {
            tableRPSEntries[i] = Dash.addTunable(
                "RPS " + (i + 1), ShooterConstants.kDefaultInterpolationTable[i][1], pos[0], pos[1]);
            advance(pos, cols);
        }

        lobShotRPSEntry = Dash.addTunable("Lob Shot RPS", ShooterConstants.kLobShotRPS, pos[0], pos[1]);
        advance(pos, cols);
    }

    /** Creates turret tunable entries on the Shooter tab. */
    public void setupTurretTunables(int[] pos, int cols) {
        turretPEntry = Dash.addTunable("Turret kP", TurretConstants.kTurretP, pos[0], pos[1]);
        advance(pos, cols);
        turretIEntry = Dash.addTunable("Turret kI", TurretConstants.kTurretI, pos[0], pos[1]);
        advance(pos, cols);
        turretDEntry = Dash.addTunable("Turret kD", TurretConstants.kTurretD, pos[0], pos[1]);
        advance(pos, cols);
        turretFFEntry = Dash.addTunable("Turret kF", TurretConstants.kTurretFF, pos[0], pos[1]);
        advance(pos, cols);
        trackingToleranceEntry = Dash.addTunable("Track Tol (deg)", TurretTrackingConstants.kTrackingToleranceDeg, pos[0], pos[1]);
        advance(pos, cols);
        potAtZeroEntry = Dash.addTunable("Pot at 0°", TurretConstants.kTurretPotAtZeroDeg, pos[0], pos[1]);
        advance(pos, cols);
        potAtMaxEntry = Dash.addTunable("Pot at 180°", TurretConstants.kTurretPotAtMaxDeg, pos[0], pos[1]);
        advance(pos, cols);
        Dash.add("Pot Raw (deg)", turret::getRawPotDegrees, pos[0], pos[1]);
        advance(pos, cols);
    }

    /** Creates intake tunable entries on the Intake tab. */
    public void setupIntakeTunables() {
        Dash.useTab("Intake");

        intakeArmPEntry = Dash.addTunable("kP", IntakeConstants.kArmP, 0, 2);
        intakeArmIEntry = Dash.addTunable("kI", IntakeConstants.kArmI, 1, 2);
        intakeArmDEntry = Dash.addTunable("kD", IntakeConstants.kArmD, 2, 2);
        intakeArmKGEntry = Dash.addTunable("kG", IntakeConstants.kArmKG, 3, 2);
        intakeArmKSEntry = Dash.addTunable("kS", IntakeConstants.kArmKS, 4, 2);
        intakeArmKVEntry = Dash.addTunable("kV", IntakeConstants.kArmKV, 5, 2);
        intakeArmKAEntry = Dash.addTunable("kA", IntakeConstants.kArmKA, 6, 2);

        cruiseVelocityEntry = Dash.addTunable("Cruise Vel (RPS)", IntakeConstants.kArmCruiseVelocity, 0, 3);
        accelerationEntry = Dash.addTunable("Accel (RPS²)", IntakeConstants.kArmAcceleration, 1, 3);
        // Lowered/raised positions are in degrees (raw throughbore reading at each stop).
        // Calibration: park arm at the stop, read "Arm Encoder (deg)" on the Intake tab,
        // type that value here. No redeploy needed — the setter is called each cycle.
        loweredPositionEntry = Dash.addTunable("Lowered Pos (deg)", IntakeConstants.kArmLoweredPosition, 2, 3);
        raisedPositionEntry = Dash.addTunable("Raised Pos (deg)", IntakeConstants.kArmRaisedPosition, 3, 3);

        // Force-write code constants to override stale Shuffleboard cache
        intakeArmPEntry.setDouble(IntakeConstants.kArmP);
        intakeArmIEntry.setDouble(IntakeConstants.kArmI);
        intakeArmDEntry.setDouble(IntakeConstants.kArmD);
        intakeArmKGEntry.setDouble(IntakeConstants.kArmKG);
        intakeArmKSEntry.setDouble(IntakeConstants.kArmKS);
        intakeArmKVEntry.setDouble(IntakeConstants.kArmKV);
        intakeArmKAEntry.setDouble(IntakeConstants.kArmKA);
        cruiseVelocityEntry.setDouble(IntakeConstants.kArmCruiseVelocity);
        accelerationEntry.setDouble(IntakeConstants.kArmAcceleration);
        loweredPositionEntry.setDouble(IntakeConstants.kArmLoweredPosition);
        raisedPositionEntry.setDouble(IntakeConstants.kArmRaisedPosition);

        // Pit calibration helpers: capture the current encoder reading into the
        // lowered or raised tunable. Park the arm at the physical stop, press
        // the button — no typing, no redeploy.
        Dash.addCommand("Capture Lowered", Commands.runOnce(() -> {
            loweredPositionEntry.setDouble(intake.getIntakeArmPosition());
        }).withName("Capture Lowered"), 4, 3);

        Dash.addCommand("Capture Raised", Commands.runOnce(() -> {
            raisedPositionEntry.setDouble(intake.getIntakeArmPosition());
        }).withName("Capture Raised"), 5, 3);

        // Tuning workflow commands
        Dash.addCommand("Zero Tunables", Commands.runOnce(() -> {
            intakeArmPEntry.setDouble(0);
            intakeArmIEntry.setDouble(0);
            intakeArmDEntry.setDouble(0);
            intakeArmKGEntry.setDouble(0);
            intakeArmKSEntry.setDouble(0);
            intakeArmKVEntry.setDouble(0);
            intakeArmKAEntry.setDouble(0);
            cruiseVelocityEntry.setDouble(0);
            accelerationEntry.setDouble(0);
            loweredPositionEntry.setDouble(0);
            raisedPositionEntry.setDouble(0);
        }).withName("Zero Tunables"), 6, 3);

        Dash.addCommand("Reset Tunables", Commands.runOnce(() -> {
            intakeArmPEntry.setDouble(IntakeConstants.kArmP);
            intakeArmIEntry.setDouble(IntakeConstants.kArmI);
            intakeArmDEntry.setDouble(IntakeConstants.kArmD);
            intakeArmKGEntry.setDouble(IntakeConstants.kArmKG);
            intakeArmKSEntry.setDouble(IntakeConstants.kArmKS);
            intakeArmKVEntry.setDouble(IntakeConstants.kArmKV);
            intakeArmKAEntry.setDouble(IntakeConstants.kArmKA);
            cruiseVelocityEntry.setDouble(IntakeConstants.kArmCruiseVelocity);
            accelerationEntry.setDouble(IntakeConstants.kArmAcceleration);
            loweredPositionEntry.setDouble(IntakeConstants.kArmLoweredPosition);
            raisedPositionEntry.setDouble(IntakeConstants.kArmRaisedPosition);
        }).withName("Reset Tunables"), 7, 3);

        Dash.useDefaultTab();
    }

    /** Creates hopper tunable entries on the Hopper tab. */
    public void setupHopperTunables() {
        Dash.useTab("Hopper");

        hopperSpeedEntry = Dash.addTunable("Hopper Speed", HopperConstants.kHopperSpeed, 0, 2);
        uptakeSpeedEntry = Dash.addTunable("Uptake Speed", HopperConstants.kUptakeSpeed, 1, 2);

        Dash.useDefaultTab();
    }

    /** Creates vision tunable entries. Call within an existing Dash.useTab() context. */
    public void setupVisionTunables(int startCol, int row) {
        int vc = startCol;
        visionModeEntry = Dash.addTunable("Vision Mode (0=Cam,1=Pose,2=Hybrid)",
            (double) VisionConstants.kVisionMode, vc++, row);
        ambiguityEntry = Dash.addTunable("Ambiguity Threshold",
            VisionConstants.kAmbiguityThreshold, vc++, row);
    }

    // ==================== Periodic ====================

    public void periodic() {
        if (Constants.SHOOTER_TAB) {
            applyShooterTunables();
            applyTurretTunables();
        }
        if (Constants.INTAKE_TAB)  applyIntakeTunables();
        if (Constants.HOPPER_TAB)  applyHopperTunables();
        if (Constants.VISION_TAB)  applyVisionTunables();
    }

    private void applyShooterTunables() {
        double p = flywheelPEntry.getDouble(ShooterConstants.kFlywheelP);
        double i = flywheelIEntry.getDouble(ShooterConstants.kFlywheelI);
        double d = flywheelDEntry.getDouble(ShooterConstants.kFlywheelD);
        double f = flywheelFFEntry.getDouble(ShooterConstants.kFlywheelFF);

        if (p != lastFlywheelP || i != lastFlywheelI || d != lastFlywheelD || f != lastFlywheelFF) {
            shooter.setFlywheelPID(p, i, d, f);
            lastFlywheelP = p;
            lastFlywheelI = i;
            lastFlywheelD = d;
            lastFlywheelFF = f;
        }

        shooter.setFlywheelTolerance(flywheelToleranceEntry.getDouble(ShooterConstants.kFlywheelToleranceRPS));
        shooter.setLobShotRPS(lobShotRPSEntry.getDouble(ShooterConstants.kLobShotRPS));

        // Rebuild interpolation table only when entries change
        boolean tableChanged = false;
        for (int idx = 0; idx < 5; idx++) {
            double dist = tableDistanceEntries[idx].getDouble(ShooterConstants.kDefaultInterpolationTable[idx][0]);
            double rps = tableRPSEntries[idx].getDouble(ShooterConstants.kDefaultInterpolationTable[idx][1]);
            if (dist != lastTableDist[idx] || rps != lastTableRPS[idx]) {
                lastTableDist[idx] = dist;
                lastTableRPS[idx] = rps;
                tableChanged = true;
            }
        }
        if (tableChanged) {
            double[][] entries = new double[5][2];
            for (int idx = 0; idx < 5; idx++) {
                entries[idx][0] = lastTableDist[idx];
                entries[idx][1] = lastTableRPS[idx];
            }
            shooter.rebuildInterpolationTable(entries);
        }
    }

    private void applyTurretTunables() {
        double p = turretPEntry.getDouble(TurretConstants.kTurretP);
        double i = turretIEntry.getDouble(TurretConstants.kTurretI);
        double d = turretDEntry.getDouble(TurretConstants.kTurretD);
        double f = turretFFEntry.getDouble(TurretConstants.kTurretFF);

        if (p != lastTurretP || i != lastTurretI || d != lastTurretD || f != lastTurretFF) {
            turret.setTurretPID(p, i, d, f);
            lastTurretP = p;
            lastTurretI = i;
            lastTurretD = d;
            lastTurretFF = f;
        }

        turret.setTrackingTolerance(trackingToleranceEntry.getDouble(TurretTrackingConstants.kTrackingToleranceDeg));
        turret.setPotAtZeroDeg(potAtZeroEntry.getDouble(TurretConstants.kTurretPotAtZeroDeg));
        turret.setPotAtMaxDeg(potAtMaxEntry.getDouble(TurretConstants.kTurretPotAtMaxDeg));
    }

    private void applyIntakeTunables() {
        double p = intakeArmPEntry.getDouble(IntakeConstants.kArmP);
        double i = intakeArmIEntry.getDouble(IntakeConstants.kArmI);
        double d = intakeArmDEntry.getDouble(IntakeConstants.kArmD);
        double kG = intakeArmKGEntry.getDouble(IntakeConstants.kArmKG);
        double kS = intakeArmKSEntry.getDouble(IntakeConstants.kArmKS);
        double kV = intakeArmKVEntry.getDouble(IntakeConstants.kArmKV);
        double kA = intakeArmKAEntry.getDouble(IntakeConstants.kArmKA);

        if (p != lastIntakeP || i != lastIntakeI || d != lastIntakeD ||
                kG != lastIntakeKG || kS != lastIntakeKS || kV != lastIntakeKV || kA != lastIntakeKA) {
            intake.setArmPID(p, i, d, kG, kS, kV, kA);
            lastIntakeP = p;
            lastIntakeI = i;
            lastIntakeD = d;
            lastIntakeKG = kG;
            lastIntakeKS = kS;
            lastIntakeKV = kV;
            lastIntakeKA = kA;
        }

        double cruiseVel = cruiseVelocityEntry.getDouble(IntakeConstants.kArmCruiseVelocity);
        double accel = accelerationEntry.getDouble(IntakeConstants.kArmAcceleration);
        if (cruiseVel != lastCruiseVelocity || accel != lastAcceleration) {
            intake.setMotionMagic(cruiseVel, accel);
            lastCruiseVelocity = cruiseVel;
            lastAcceleration = accel;
        }

        intake.setArmLoweredPosition(loweredPositionEntry.getDouble(IntakeConstants.kArmLoweredPosition));
        intake.setArmRaisedPosition(raisedPositionEntry.getDouble(IntakeConstants.kArmRaisedPosition));
    }

    private void applyHopperTunables() {
        hopper.setHopperSpeed(hopperSpeedEntry.getDouble(HopperConstants.kHopperSpeed));
        hopper.setUptakeSpeed(uptakeSpeedEntry.getDouble(HopperConstants.kUptakeSpeed));
    }

    private void applyVisionTunables() {
        if (vision == null || visionModeEntry == null) return;
        vision.setVisionMode((int) visionModeEntry.getDouble(VisionConstants.kVisionMode));
        vision.setAmbiguityThreshold(ambiguityEntry.getDouble(VisionConstants.kAmbiguityThreshold));
    }

    // ==================== Helpers ====================

    private static void advance(int[] pos, int cols) {
        pos[0]++;
        if (pos[0] >= cols) {
            pos[0] = 0;
            pos[1]++;
        }
    }

    private static void newRow(int[] pos) {
        if (pos[0] != 0) {
            pos[0] = 0;
            pos[1]++;
        }
    }
}
