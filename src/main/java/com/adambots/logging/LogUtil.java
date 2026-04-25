// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.logging;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.adambots.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;

/**
 * Level-based wrapper around AdvantageKit's {@link Logger#recordOutput}.
 *
 * <p>Replaces the prior single-boolean {@code LOGGING_ENABLED} / {@code CURRENT_LOGGING}
 * gates. Each log call tags itself with a {@link Level}; calls below the compile-time
 * {@link Constants#LOG_LEVEL} floor are dead-code-eliminated by the JIT — zero cost
 * at runtime when the level is disabled.
 *
 * <h3>Usage (static import recommended)</h3>
 * <pre>{@code
 * import static com.adambots.logging.LogUtil.*;
 *
 * // Match-critical — always logged in comp:
 * log(ESSENTIAL, "Shooter/TargetRPS", target);
 *
 * // Subsystem diagnostics — on at bench / practice, off at quals:
 * log(DIAGNOSTIC, "Vision/CamEWMAAngle", ewmaAngle);
 *
 * // High-rate tuning data — bench only:
 * log(DEBUG, "Turret/LeadX", leadX);
 *
 * // Supplier form avoids the expensive computation when level is disabled:
 * log(DEBUG, "Vision/PoseAngleTurretRelative", () -> poseAngleToTurretRelative(angle));
 * }</pre>
 *
 * <h3>Important notes</h3>
 * <ul>
 *   <li>{@code Constants.LOG_LEVEL} is an enum (not inlined); regular
 *       {@code ./gradlew deploy} picks up changes. Gradle's incremental compiler
 *       tracks ABI changes so constant-only edits recompile dependents correctly.
 *       Use {@code clean} only when switching vendor deps or annotation-processor
 *       output goes stale.</li>
 *   <li>{@code Logger.recordOutput} is thread-safe per AdvantageKit docs. Safe to call
 *       from PathPlanner callbacks or custom threads.</li>
 *   <li>Supplier overloads allocate a small lambda; use them only for computations
 *       that are themselves expensive (vision math, Pose2d constructors), not for
 *       primitives already in scope.</li>
 * </ul>
 *
 * <h3>Level guidelines</h3>
 * <ul>
 *   <li><b>NONE</b> — kill-switch. Set {@code Constants.LOG_LEVEL = NONE} to disable
 *       every team-side {@code log(...)} call. Useful as a diagnostic to isolate the
 *       cost of logging from the cost of the work being logged.</li>
 *   <li><b>ESSENTIAL</b> — must-have for post-match forensics (currents, target
 *       setpoints, at-speed flags, PathPlanner target/current pose, scheduler timing,
 *       subsystem positions). ~25–30 signals total.</li>
 *   <li><b>DIAGNOSTIC</b> — subsystem internals that help tune / troubleshoot but
 *       aren't needed for match replay (EWMA filters, raw-vs-filtered pairs,
 *       rejection counters, per-module swerve outputs).</li>
 *   <li><b>DEBUG</b> — high-rate / high-cardinality tuning data (lead-comp math,
 *       PID internals, per-tag vision observations).</li>
 * </ul>
 */
public final class LogUtil {

    public enum Level {
        /**
         * Kill-switch level. Set {@code Constants.LOG_LEVEL = Level.NONE} to disable
         * every {@code log(...)} call in team code — useful as a diagnostic to measure
         * the cost of logging itself. AdvantageKit's auto-logged signals (DriverStation,
         * SystemStats, PowerDistribution) and NT passthrough still flow regardless of
         * this setting; see {@code Robot()} data-receiver configuration to disable those.
         *
         * <p>Not intended as a value to pass into {@code log(...)} call sites — only
         * as a value for {@code Constants.LOG_LEVEL}.
         */
        NONE,
        ESSENTIAL,
        DIAGNOSTIC,
        DEBUG;

        /** True if {@code Constants.LOG_LEVEL} permits this level. NONE is a sentinel
         *  kill-switch and is never "enabled" as a call-site level. */
        public boolean enabled() {
            if (this == NONE) return false;
            return this.ordinal() <= Constants.LOG_LEVEL.ordinal();
        }
    }

    // Static-import convenience: `import static com.adambots.logging.LogUtil.*;`
    // lets call sites write `log(ESSENTIAL, ...)` instead of `log(Level.ESSENTIAL, ...)`.
    public static final Level NONE       = Level.NONE;
    public static final Level ESSENTIAL  = Level.ESSENTIAL;
    public static final Level DIAGNOSTIC = Level.DIAGNOSTIC;
    public static final Level DEBUG      = Level.DEBUG;

    private LogUtil() {
        throw new UnsupportedOperationException("Utility class");
    }

    // ==================== Primitive overloads ====================

    public static void log(Level l, String key, double v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, float v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, long v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, int v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, boolean v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, String v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    // ==================== Array overloads ====================

    public static void log(Level l, String key, double[] v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, String[] v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    // ==================== Struct overloads ====================

    public static void log(Level l, String key, Pose2d v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, Pose2d[] v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    public static void log(Level l, String key, Pose3d v) {
        if (l.enabled()) Logger.recordOutput(key, v);
    }

    // ==================== Supplier overloads ====================
    // Use these when computing the value is expensive AND the level may be disabled.
    // For primitives already in scope, use the direct overloads instead.

    public static void log(Level l, String key, DoubleSupplier s) {
        if (l.enabled()) Logger.recordOutput(key, s.getAsDouble());
    }

    public static void log(Level l, String key, BooleanSupplier s) {
        if (l.enabled()) Logger.recordOutput(key, s.getAsBoolean());
    }

    public static <T> void log(Level l, String key, Class<T> type, Supplier<T> s) {
        if (l.enabled()) {
            T val = s.get();
            if (val instanceof Pose2d p)          Logger.recordOutput(key, p);
            else if (val instanceof Pose3d p)     Logger.recordOutput(key, p);
            else if (val instanceof Pose2d[] arr) Logger.recordOutput(key, arr);
            else if (val instanceof String str)   Logger.recordOutput(key, str);
            // Primitive suppliers should use the typed overloads above for zero allocation.
        }
    }
}
