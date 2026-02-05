// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.test;

import com.adambots.Constants.TestTurretConstants;
import com.adambots.lib.utils.tuning.PIDAutoTuner;
import com.adambots.lib.utils.tuning.TuningResult;
import com.adambots.subsystems.test.TestTurretSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Factory class for PIDAutoTuner test commands.
 *
 * <p>This class provides commands for testing the PIDAutoTuner from AdambotsLib:
 * <ul>
 *   <li>{@link #tuneTurretPosition(TestTurretSubsystem)} - Run auto-tuning on the test turret</li>
 *   <li>{@link #applyTunedGains(TestTurretSubsystem, String)} - Apply tuned gains to the turret</li>
 *   <li>{@link #testTunedPerformance(TestTurretSubsystem, double)} - Test turret with tuned gains</li>
 * </ul>
 *
 * <p>Usage in RobotContainer:
 * <pre>
 * // Bind to buttons for testing
 * Buttons.JoystickButton11.onTrue(PIDTunerTestCommands.tuneTurretPosition(testTurret));
 * Buttons.JoystickButton12.onTrue(PIDTunerTestCommands.testTunedPerformance(testTurret, 90));
 * </pre>
 */
public final class PIDTunerTestCommands {

    private static final String TUNER_NAME = "TestTurret";

    // Private constructor to prevent instantiation
    private PIDTunerTestCommands() {}

    /**
     * Creates a command that auto-tunes the turret position PID.
     *
     * <p>Uses the Relay Feedback Method (Ziegler-Nichols) to determine optimal PID gains.
     * The turret will oscillate during tuning - watch the Mechanism2d visualization
     * to see the relay oscillations as the tuner finds Ku and Tu.
     *
     * <p>After tuning completes:
     * <ul>
     *   <li>Results are displayed on SmartDashboard under "PID Tuning"</li>
     *   <li>Use {@link #applyTunedGains} to apply the results</li>
     *   <li>Results can be retrieved with {@code PIDAutoTuner.getLastResult("TestTurret")}</li>
     * </ul>
     *
     * @param turret The test turret subsystem
     * @return Command that runs the auto-tuning process
     */
    public static Command tuneTurretPosition(TestTurretSubsystem turret) {
        return Commands.sequence(
            // Reset oscillation detection before tuning
            Commands.runOnce(() -> {
                turret.resetOscillationDetection();
                System.out.println("\n========================================");
                System.out.println("PID AUTO-TUNER: " + TUNER_NAME);
                System.out.println("========================================");
                System.out.println("Starting turret position tuning...");
                System.out.println("Watch the Mechanism2d visualization in SmartDashboard");
                System.out.println("The turret will oscillate as the tuner finds Ku and Tu");
            }),

            // Run the PIDAutoTuner
            PIDAutoTuner.tunePosition(
                TUNER_NAME,
                turret::getAngle,
                turret::setPercentOutput,
                TestTurretConstants.kMinAngle,
                TestTurretConstants.kMaxAngle
            ),

            // Print results
            Commands.runOnce(() -> {
                TuningResult result = PIDAutoTuner.getLastResult(TUNER_NAME);
                if (result != null) {
                    System.out.println("\n========================================");
                    System.out.println("TUNING COMPLETE: " + TUNER_NAME);
                    System.out.println("----------------------------------------");
                    System.out.println(result.toCodeString());
                    System.out.println("========================================\n");

                    // Publish to dashboard
                    SmartDashboard.putNumber("PID Tuning/" + TUNER_NAME + "/Tuned kP", result.kP());
                    SmartDashboard.putNumber("PID Tuning/" + TUNER_NAME + "/Tuned kI", result.kI());
                    SmartDashboard.putNumber("PID Tuning/" + TUNER_NAME + "/Tuned kD", result.kD());
                    SmartDashboard.putString("PID Tuning/" + TUNER_NAME + "/Status", "Tuning Complete");
                } else {
                    System.out.println("ERROR: No tuning result available for " + TUNER_NAME);
                    SmartDashboard.putString("PID Tuning/" + TUNER_NAME + "/Status", "Tuning Failed");
                }
            })
        ).withName("TuneTurretPosition");
    }

    /**
     * Creates a command that applies the last tuned gains to the turret.
     *
     * <p>This retrieves the tuning result from PIDAutoTuner and applies it to:
     * <ul>
     *   <li>The subsystem's internal PID controller</li>
     *   <li>The motor's onboard PID slot (if supported)</li>
     * </ul>
     *
     * @param turret The test turret subsystem
     * @param name The name used during tuning (e.g., "TestTurret")
     * @return Command that applies the tuned gains
     */
    public static Command applyTunedGains(TestTurretSubsystem turret, String name) {
        return Commands.runOnce(() -> {
            TuningResult result = PIDAutoTuner.getLastResult(name);
            if (result != null) {
                // Apply to subsystem's PID controller
                turret.setPIDGains(result.kP(), result.kI(), result.kD());

                // Apply to motor's onboard PID (slot 0)
                result.applyTo(turret.getMotor(), 0);

                System.out.println("Applied tuned gains to " + name + ":");
                System.out.println("  kP=" + result.kP() + ", kI=" + result.kI() + ", kD=" + result.kD());
                SmartDashboard.putString("PID Tuning/" + name + "/Status", "Gains Applied");
            } else {
                System.out.println("ERROR: No tuning result available for " + name);
                System.out.println("Run tuning first with tuneTurretPosition()");
                SmartDashboard.putString("PID Tuning/" + name + "/Status", "No Results");
            }
        }).withName("ApplyTunedGains(" + name + ")");
    }

    /**
     * Creates a command that tests the turret performance with tuned gains.
     *
     * <p>This command:
     * <ol>
     *   <li>Applies the last tuned gains (if available)</li>
     *   <li>Commands the turret to the specified angle</li>
     *   <li>Waits for the turret to reach the target</li>
     *   <li>Measures and reports settling time</li>
     * </ol>
     *
     * <p>Watch the Mechanism2d visualization to see how well the tuned PID performs.
     * A well-tuned system should:
     * <ul>
     *   <li>Reach the target quickly without excessive overshoot</li>
     *   <li>Settle without oscillation</li>
     *   <li>Hold position accurately</li>
     * </ul>
     *
     * @param turret The test turret subsystem
     * @param targetAngle The target angle in degrees to test
     * @return Command that tests the tuned performance
     */
    public static Command testTunedPerformance(TestTurretSubsystem turret, double targetAngle) {
        final double[] startTime = {0};

        return Commands.sequence(
            // Apply tuned gains first
            applyTunedGains(turret, TUNER_NAME),

            // Record start time
            Commands.runOnce(() -> {
                startTime[0] = System.currentTimeMillis() / 1000.0;
                System.out.println("\n--- Testing Tuned Performance ---");
                System.out.println("Target angle: " + targetAngle + " degrees");
                System.out.println("Starting from: " + turret.getAngle() + " degrees");
                SmartDashboard.putString("PID Tuning/" + TUNER_NAME + "/Status", "Testing...");
            }),

            // Command turret to target and wait
            turret.aimToAngleCommand(targetAngle)
                .until(turret.isAtTargetTrigger())
                .withTimeout(5.0),  // 5 second timeout

            // Report results
            Commands.runOnce(() -> {
                double settlingTime = (System.currentTimeMillis() / 1000.0) - startTime[0];
                double finalAngle = turret.getAngle();
                double error = Math.abs(finalAngle - targetAngle);

                System.out.println("\n--- Performance Results ---");
                System.out.println("Settling time: " + String.format("%.3f", settlingTime) + " seconds");
                System.out.println("Final angle: " + String.format("%.2f", finalAngle) + " degrees");
                System.out.println("Steady-state error: " + String.format("%.2f", error) + " degrees");

                // Check if target was reached (within 2 degree tolerance)
                // Don't use isAtTargetTrigger() because pidEnabled is false after command ends
                if (error < 2.0) {
                    System.out.println("Status: SUCCESS - Target reached!");
                    SmartDashboard.putString("PID Tuning/" + TUNER_NAME + "/Status", "Test Passed");
                } else {
                    System.out.println("Status: FAILED - Target not reached (error: " + String.format("%.2f", error) + " deg)");
                    SmartDashboard.putString("PID Tuning/" + TUNER_NAME + "/Status", "Test Failed");
                }

                SmartDashboard.putNumber("PID Tuning/" + TUNER_NAME + "/SettlingTime", settlingTime);
                SmartDashboard.putNumber("PID Tuning/" + TUNER_NAME + "/SteadyStateError", error);
            })
        ).withName("TestTunedPerformance(" + targetAngle + ")");
    }

    /**
     * Creates a command that runs a comprehensive test sequence.
     *
     * <p>This command tests the turret at multiple angles to verify
     * consistent performance across the range of motion.
     *
     * @param turret The test turret subsystem
     * @return Command that runs a multi-angle test
     */
    public static Command runComprehensiveTest(TestTurretSubsystem turret) {
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("\n=== Starting Comprehensive Test ===")),
            testTunedPerformance(turret, 45),
            Commands.waitSeconds(1.0),
            testTunedPerformance(turret, -45),
            Commands.waitSeconds(1.0),
            testTunedPerformance(turret, 90),
            Commands.waitSeconds(1.0),
            testTunedPerformance(turret, -90),
            Commands.waitSeconds(1.0),
            testTunedPerformance(turret, 0),
            Commands.runOnce(() -> System.out.println("\n=== Comprehensive Test Complete ==="))
        ).withName("ComprehensiveTurretTest");
    }
}
