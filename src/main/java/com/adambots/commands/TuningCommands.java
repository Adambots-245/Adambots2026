package com.adambots.commands;

import com.adambots.Constants.TuningConstants;
import com.adambots.lib.subsystems.SwerveSubsystem;
import com.adambots.lib.utils.Dash;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Command factory for swerve drive auto-tuning routines.
 *
 * <p>PID tuning uses a <b>critical gain search</b>: apply P-only control at increasing
 * gains until the system oscillates, then compute PID via Ziegler-Nichols.
 *
 * <p>MOI estimation uses a step-response spin-up test.
 */
public final class TuningCommands {

    /** Seconds to run each P-gain test iteration */
    private static final double TEST_DURATION = 5.0;
    /** Starting proportional gain for the search */
    private static final double STARTING_KP = 1.0;
    /** Maximum gain-doubling iterations before giving up */
    private static final int MAX_ITERATIONS = 8;
    /** Number of error zero-crossings to declare oscillation */
    private static final int OSCILLATION_THRESHOLD = 3;
    /** Seconds to ignore at the start of each iteration (initial approach phase) */
    private static final double APPROACH_BLANKING = 0.5;

    /**
     * Safe spawn position — navgrid row 7 (y=2.1m), column 10 (x=3.0m).
     * Verified clear of all structures in the 2026 REBUILT field.
     */
    private static final Pose2d SAFE_POSE = new Pose2d(3.0, 2.1, Rotation2d.fromDegrees(0));

    private TuningCommands() {}

    // ======================== TRANSLATION PID ========================

    /**
     * Tunes the translation (position) PID by searching for the critical proportional gain.
     * Robot oscillates forward/back ~1m. Results on "PID Tuning" Shuffleboard tab.
     *
     * @param swerve The swerve drive subsystem
     * @return Command that performs translation PID tuning
     */
    public static Command tuneTranslationPIDCommand(SwerveSubsystem swerve) {
        double maxVel = swerve.getSwerveDrive().getMaximumChassisVelocity();
        double maxDriveSpeed = maxVel * TuningConstants.kTuningMaxLinearOutput;
        double stepDistance = 1.0;

        final Translation2d[] refPos = {null};
        final double[] refHeading = new double[1];
        final double[] testKp = new double[1];
        final double[] iterStart = new double[1];
        final double[] prevError = new double[1];
        final int[] crossings = new int[1];
        final double[] firstCrossTime = new double[1];
        final double[] lastCrossTime = new double[1];
        final int[] iter = new int[1];
        final boolean[] done = {false};
        final double[] ku = new double[1];
        final double[] tu = new double[1];
        final double[] stepDir = {1.0};
        final int[] cycleCount = new int[1];
        final double[] warmupStart = new double[1];

        // Note: every Commands.run/runOnce passes `swerve` so the scheduler
        // knows this command requires the swerve subsystem and interrupts the default command.
        return Commands.sequence(
            // Phase 0: Reset pose (sim only — on real robot, keep actual odometry)
            Commands.runOnce(() -> {
                if (RobotBase.isSimulation()) {
                    swerve.resetOdometry(SAFE_POSE);
                }
                refPos[0] = swerve.getPose().getTranslation();
                refHeading[0] = swerve.getHeading().getRadians();
                testKp[0] = STARTING_KP;
                iter[0] = 0;
                iterStart[0] = 0;
                done[0] = false;
                cycleCount[0] = 0;
                warmupStart[0] = Timer.getFPGATimestamp();
                System.out.println("\n========================================");
                System.out.println("TRANSLATION PID: Critical Gain Search");
                System.out.println("  maxVel=" + String.format("%.1f", maxVel)
                    + " m/s, maxDriveSpeed=" + String.format("%.1f", maxDriveSpeed) + " m/s");
                System.out.println("  Starting pose: " + swerve.getPose());
                System.out.println("========================================");
            }, swerve),

            // Phase 1: Drive forward at constant speed for 1s to verify movement
            Commands.run(() -> {
                swerve.drive(new Translation2d(maxDriveSpeed, 0), 0, false);
                double elapsed = Timer.getFPGATimestamp() - warmupStart[0];
                if (elapsed < 0.1 || (elapsed > 0.5 && elapsed < 0.52) || (elapsed > 0.9 && elapsed < 0.92)) {
                    System.out.println("  [Warmup t=" + String.format("%.2f", elapsed) + "s] pose=" + swerve.getPose());
                }
            }, swerve).withTimeout(1.0).withName("TranslationWarmup"),

            // Phase 1.5: Check warmup result and reset reference
            Commands.runOnce(() -> {
                Pose2d warmupPose = swerve.getPose();
                double dist = warmupPose.getTranslation().getDistance(refPos[0]);
                System.out.println("  [Warmup done] Moved " + String.format("%.3f", dist) + "m | pose=" + warmupPose);
                if (dist < 0.05) {
                    System.out.println("  *** WARNING: Robot did NOT move during warmup! ***");
                }
                // Reset reference for the actual tuning iterations
                refPos[0] = swerve.getPose().getTranslation();
                refHeading[0] = swerve.getHeading().getRadians();
            }, swerve),

            // Phase 2: Critical gain search
            Commands.run(() -> {
                double now = Timer.getFPGATimestamp();
                double h = refHeading[0];

                double dx = swerve.getPose().getX() - refPos[0].getX();
                double dy = swerve.getPose().getY() - refPos[0].getY();
                double displacement = dx * Math.cos(h) + dy * Math.sin(h);
                double target = stepDir[0] * stepDistance;
                double error = target - displacement;

                // Start new iteration when needed
                if (iterStart[0] == 0 || now - iterStart[0] > TEST_DURATION) {
                    if (iterStart[0] > 0) {
                        System.out.println("    → " + crossings[0] + " zero-crossings"
                            + " | pose=" + swerve.getPose());
                        if (crossings[0] >= OSCILLATION_THRESHOLD) {
                            ku[0] = testKp[0];
                            tu[0] = (crossings[0] > 1)
                                ? 2.0 * (lastCrossTime[0] - firstCrossTime[0]) / (crossings[0] - 1)
                                : 0.5;
                            done[0] = true;
                            return;
                        }
                        testKp[0] *= 2.0;
                    }

                    iter[0]++;
                    if (iter[0] > MAX_ITERATIONS) {
                        ku[0] = testKp[0];
                        tu[0] = 0.5;
                        done[0] = true;
                        return;
                    }

                    refPos[0] = swerve.getPose().getTranslation();
                    refHeading[0] = swerve.getHeading().getRadians();
                    stepDir[0] = (iter[0] % 2 == 0) ? -1.0 : 1.0;
                    iterStart[0] = now;
                    crossings[0] = 0;
                    prevError[0] = stepDir[0] * stepDistance;
                    firstCrossTime[0] = 0;
                    lastCrossTime[0] = 0;
                    System.out.println("  [Iter " + iter[0] + "] kP="
                        + String.format("%.1f", testKp[0]));
                }

                // P-only control
                double velocity = MathUtil.clamp(
                    testKp[0] * error, -maxDriveSpeed, maxDriveSpeed);
                swerve.drive(new Translation2d(velocity, 0), 0, false);

                // Diagnostic: log first 5 cycles of each iteration
                cycleCount[0]++;
                if (cycleCount[0] <= 5 || cycleCount[0] % 50 == 0) {
                    System.out.println("    cycle=" + cycleCount[0]
                        + " err=" + String.format("%.3f", error)
                        + " vel=" + String.format("%.3f", velocity)
                        + " pose=" + swerve.getPose());
                }

                // Detect zero crossings after approach phase
                if (now - iterStart[0] > APPROACH_BLANKING) {
                    if (prevError[0] * error < 0 && Math.abs(error) > 0.01) {
                        crossings[0]++;
                        if (crossings[0] == 1) firstCrossTime[0] = now;
                        lastCrossTime[0] = now;
                    }
                }
                prevError[0] = error;
            }, swerve).until(() -> done[0]),

            Commands.runOnce(() -> {
                swerve.drive(new Translation2d(0, 0), 0, false);
                reportPIDResults("Translation", ku[0], tu[0]);
            }, swerve)
        ).finallyDo(() -> swerve.drive(new Translation2d(0, 0), 0, false))
         .withName("TuneTranslationPID");
    }

    // ======================== ROTATION PID ========================

    /**
     * Tunes the rotation (heading) PID by searching for the critical proportional gain.
     * Robot spins in place. Results on "PID Tuning" Shuffleboard tab.
     *
     * @param swerve The swerve drive subsystem
     * @return Command that performs rotation PID tuning
     */
    public static Command tuneRotationPIDCommand(SwerveSubsystem swerve) {
        double maxAngVel = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
        double maxOmega = maxAngVel * TuningConstants.kTuningMaxAngularOutput;
        double stepAngle = Math.toRadians(45);

        final double[] targetHeading = new double[1];
        final double[] testKp = new double[1];
        final double[] iterStart = new double[1];
        final double[] prevError = new double[1];
        final int[] crossings = new int[1];
        final double[] firstCrossTime = new double[1];
        final double[] lastCrossTime = new double[1];
        final int[] iter = new int[1];
        final boolean[] done = {false};
        final double[] ku = new double[1];
        final double[] tu = new double[1];
        final double[] stepDir = {1.0};
        final int[] cycleCount = new int[1];
        final double[] warmupStart = new double[1];

        return Commands.sequence(
            Commands.runOnce(() -> {
                if (RobotBase.isSimulation()) {
                    swerve.resetOdometry(SAFE_POSE);
                }
                testKp[0] = STARTING_KP;
                iter[0] = 0;
                iterStart[0] = 0;
                done[0] = false;
                cycleCount[0] = 0;
                warmupStart[0] = Timer.getFPGATimestamp();
                System.out.println("\n========================================");
                System.out.println("ROTATION PID: Critical Gain Search");
                System.out.println("  Starting pose: " + swerve.getPose());
                System.out.println("========================================");
            }, swerve),

            // Warmup: spin for 1s to verify rotation works
            Commands.run(() -> {
                swerve.drive(new Translation2d(0, 0), maxOmega, false);
                double elapsed = Timer.getFPGATimestamp() - warmupStart[0];
                if (elapsed < 0.1 || (elapsed > 0.5 && elapsed < 0.52) || (elapsed > 0.9 && elapsed < 0.92)) {
                    System.out.println("  [Warmup t=" + String.format("%.2f", elapsed) + "s] heading="
                        + String.format("%.1f", swerve.getHeading().getDegrees()) + "°");
                }
            }, swerve).withTimeout(1.0).withName("RotationWarmup"),

            Commands.runOnce(() -> {
                double startHeading = swerve.getHeading().getDegrees();
                System.out.println("  [Warmup done] Heading=" + String.format("%.1f", startHeading) + "°");
            }, swerve),

            Commands.run(() -> {
                double now = Timer.getFPGATimestamp();
                double currentHeading = swerve.getHeading().getRadians();

                double error = targetHeading[0] - currentHeading;
                while (error > Math.PI) error -= 2 * Math.PI;
                while (error < -Math.PI) error += 2 * Math.PI;

                if (iterStart[0] == 0 || now - iterStart[0] > TEST_DURATION) {
                    if (iterStart[0] > 0) {
                        System.out.println("    → " + crossings[0] + " zero-crossings"
                            + " | heading=" + String.format("%.1f", Math.toDegrees(currentHeading)) + "°");
                        if (crossings[0] >= OSCILLATION_THRESHOLD) {
                            ku[0] = testKp[0];
                            tu[0] = (crossings[0] > 1)
                                ? 2.0 * (lastCrossTime[0] - firstCrossTime[0]) / (crossings[0] - 1)
                                : 0.5;
                            done[0] = true;
                            return;
                        }
                        testKp[0] *= 2.0;
                    }

                    iter[0]++;
                    if (iter[0] > MAX_ITERATIONS) {
                        ku[0] = testKp[0];
                        tu[0] = 0.5;
                        done[0] = true;
                        return;
                    }

                    stepDir[0] = (iter[0] % 2 == 0) ? -1.0 : 1.0;
                    targetHeading[0] = currentHeading + stepDir[0] * stepAngle;
                    iterStart[0] = now;
                    crossings[0] = 0;
                    prevError[0] = stepDir[0] * stepAngle;
                    firstCrossTime[0] = 0;
                    lastCrossTime[0] = 0;
                    System.out.println("  [Iter " + iter[0] + "] kP="
                        + String.format("%.1f", testKp[0]));
                }

                double omega = MathUtil.clamp(testKp[0] * error, -maxOmega, maxOmega);
                swerve.drive(new Translation2d(0, 0), omega, false);

                if (now - iterStart[0] > APPROACH_BLANKING) {
                    if (prevError[0] * error < 0 && Math.abs(error) > Math.toRadians(0.5)) {
                        crossings[0]++;
                        if (crossings[0] == 1) firstCrossTime[0] = now;
                        lastCrossTime[0] = now;
                    }
                }
                prevError[0] = error;
            }, swerve).until(() -> done[0]),

            Commands.runOnce(() -> {
                swerve.drive(new Translation2d(0, 0), 0, false);
                reportPIDResults("Rotation", ku[0], tu[0]);
            }, swerve)
        ).finallyDo(() -> swerve.drive(new Translation2d(0, 0), 0, false))
         .withName("TuneRotationPID");
    }

    // ======================== MOI ESTIMATION ========================

    /**
     * Estimates MOI via step-response spin-up. Robot spins in place.
     * Results on "PID Tuning" Shuffleboard tab.
     *
     * @param swerve The swerve drive subsystem
     * @return Command that estimates MOI and logs to dashboard
     */
    public static Command estimateMOICommand(SwerveSubsystem swerve) {
        final double[] startTime = new double[1];
        final double[] prevOmega = new double[1];
        final double[] prevTime = new double[1];
        final double[] accelSum = new double[1];
        final int[] sampleCount = new int[1];
        final double[] maxOmega = new double[1];

        double targetOmega = TuningConstants.kMOITestAngularVelocity;
        double duration = TuningConstants.kMOITestDurationSeconds;
        double settleTime = TuningConstants.kMOISpinUpSettleTime;

        return Commands.sequence(
            Commands.runOnce(() -> {
                if (RobotBase.isSimulation()) {
                    swerve.resetOdometry(SAFE_POSE);
                }
                startTime[0] = Timer.getFPGATimestamp();
                prevOmega[0] = 0;
                prevTime[0] = Timer.getFPGATimestamp();
                accelSum[0] = 0;
                sampleCount[0] = 0;
                maxOmega[0] = 0;
                System.out.println("\n========================================");
                System.out.println("MOI ESTIMATION: Starting spin-up test");
                System.out.println("  Starting pose: " + swerve.getPose());
                System.out.println("========================================");
            }, swerve),

            Commands.run(() -> {
                swerve.drive(new Translation2d(0, 0), targetOmega, false);

                double now = Timer.getFPGATimestamp();
                double elapsed = now - startTime[0];
                double currentOmega = swerve.getRobotVelocity().omegaRadiansPerSecond;

                if (Math.abs(currentOmega) > Math.abs(maxOmega[0])) {
                    maxOmega[0] = currentOmega;
                }

                double dt = now - prevTime[0];
                if (dt >= TuningConstants.kMOISampleIntervalSeconds && elapsed < settleTime) {
                    double alpha = (currentOmega - prevOmega[0]) / dt;
                    if (Math.abs(alpha) > 0.01) {
                        accelSum[0] += Math.abs(alpha);
                        sampleCount[0]++;
                    }
                    prevOmega[0] = currentOmega;
                    prevTime[0] = now;
                }
            }, swerve).until(() -> Timer.getFPGATimestamp() - startTime[0] > duration),

            Commands.runOnce(() -> {
                swerve.drive(new Translation2d(0, 0), 0, false);

                if (sampleCount[0] < 3) {
                    String msg = "MOI Estimation: Insufficient acceleration samples ("
                        + sampleCount[0] + "). Try increasing test duration.";
                    DriverStation.reportError(msg, false);
                    System.out.println(msg);
                    Dash.useTab("PID Tuning");
                    Dash.addTunable("MOI Estimated (kg·m²)", -1);
                    Dash.useDefaultTab();
                    return;
                }

                double avgAlpha = accelSum[0] / sampleCount[0];
                double trackRadius = 0.3;
                double forcePerModule = 40.0;
                int numModules = 4;
                double estimatedTorque = forcePerModule * trackRadius * numModules;
                double estimatedMOI = estimatedTorque / avgAlpha;

                System.out.println("\n========================================");
                System.out.println("MOI ESTIMATION RESULTS");
                System.out.println("  Estimated MOI: " + String.format("%.2f", estimatedMOI) + " kg·m²");
                System.out.println("========================================\n");

                Dash.useTab("PID Tuning");
                Dash.addTunable("MOI Estimated (kg·m²)", estimatedMOI);
                Dash.addTunable("MOI Avg Angular Accel", avgAlpha);
                Dash.addTunable("MOI Steady-State Omega", maxOmega[0]);
                Dash.addTunable("MOI Accel Samples", sampleCount[0]);
                Dash.useDefaultTab();
            }, swerve)
        ).finallyDo(() -> swerve.drive(new Translation2d(0, 0), 0, false))
         .withName("EstimateMOI");
    }

    // ======================== RESULT REPORTING ========================

    /**
     * Computes and reports PID results from the critical gain and period.
     */
    private static void reportPIDResults(String label, double ku, double tu) {
        tu = Math.max(tu, 0.04);

        // Ziegler-Nichols classic PID (aggressive, may overshoot)
        double znKp = 0.6 * ku;
        double znKi = 1.2 * ku / tu;
        double znKd = 0.075 * ku * tu;

        // Ziegler-Nichols "no overshoot" variant (conservative)
        // kP = 0.2 * Ku, kD = 0.066 * Ku * Tu, kI = 0
        double consKp = 0.2 * ku;
        double consKd = 0.066 * ku * tu;

        // PathPlanner-scale recommendation: quarter of conservative
        // Typical PathPlanner translation PID: 1-10, rotation PID: 1-10
        double ppKp = Math.min(consKp, 10.0);
        double ppKd = Math.min(consKd, 2.0);

        System.out.println("\n========================================");
        System.out.println(label.toUpperCase() + " PID RESULTS");
        System.out.println("  Critical gain (Ku): " + String.format("%.2f", ku));
        System.out.println("  Critical period (Tu): " + String.format("%.3f", tu) + "s");
        System.out.println("  ── For PathPlanner settings.json ──");
        System.out.println("  Conservative: kP=" + String.format("%.2f", consKp)
            + " kI=0 kD=" + String.format("%.3f", consKd));
        System.out.println("  Start with:   kP=" + String.format("%.2f", ppKp)
            + " kI=0 kD=" + String.format("%.3f", ppKd));
        System.out.println("  ── Ziegler-Nichols classic (aggressive) ──");
        System.out.println("  Aggressive:  kP=" + String.format("%.2f", znKp)
            + " kI=" + String.format("%.2f", znKi) + " kD=" + String.format("%.3f", znKd));
        System.out.println("========================================\n");

        Dash.useTab("PID Tuning");
        Dash.addTunable(label + " kP (start with)", ppKp);
        Dash.addTunable(label + " kI (start with)", 0.0);
        Dash.addTunable(label + " kD (start with)", ppKd);
        Dash.addTunable(label + " kP (conservative)", consKp);
        Dash.addTunable(label + " kD (conservative)", consKd);
        Dash.addTunable(label + " Ku (critical gain)", ku);
        Dash.addTunable(label + " Tu (critical period)", tu);
        Dash.useDefaultTab();
    }
}
