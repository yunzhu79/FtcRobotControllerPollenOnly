package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Auto distancing: shooting position -> flywheel target (ticks/s), for ANY hive.
 *
 * Fitted 2026-11-26 to 10 measured min/max ranges shooting RED_1 at (59.25, 58.6).
 * Every input is measured RELATIVE to the target hive (distance and angle), so the
 * other three hives are the same surface shifted to their own (x, y). Pass the
 * target's coordinates; nothing here is specific to RED_1.
 *
 *   v = sqrt( g d^2 / (2 cos^2(theta) (d tan(theta) - h)) ) / k   +   c * angle
 *
 *   d      distance robot center -> hive (in)
 *   theta  effective launch angle
 *   h      effective height of the hive above the shooter (in)
 *   k      ball exit speed per flywheel tick/s ((in/s) per (tick/s))
 *   angle  degrees from "straight in front of the hive" (robot below it in field y):
 *          + = robot to the RIGHT of the hive (larger x), - = to the LEFT.
 *          Clamped to +/-MAX_ANGLE_DEG so the angle term is never extrapolated far.
 *   c      ticks/s per degree (left/right correction)
 *
 * The speed is lowest around 49 in and rises both closer and farther.
 *   USE_ANGLE_TERM = false  -> distance only (symmetric left/right)
 *   USE_ANGLE_TERM = true   -> adds the left/right term (right side needs more speed)
 *
 * Measured data covers about 44-70 in and -46..+47 deg. isTrusted() is false outside
 * MIN/MAX_TRUSTED_DISTANCE_IN or beyond MAX_ANGLE_DEG, so OpModes can warn the driver.
 */
public class AutoDistance {
    public static final double G_IN_S2 = 386.1;

    // ---- Distance-only model ----
    public static double THETA_RAD = 0.823;     // ~47.2 deg
    public static double H_IN = 26.578;
    public static double K = 0.255;

    // ---- Distance + angle model ----
    public static double THETA_ANGLE_RAD = 0.822;
    public static double H_ANGLE_IN = 26.928;
    public static double K_ANGLE = 0.258;
    public static double C_TICKS_PER_DEG = 0.162;

    /** false = distance only, true = distance + left/right angle. */
    public static boolean USE_ANGLE_TERM = false;

    public static double MIN_VELOCITY = 650.0;
    public static double MAX_VELOCITY = 900.0;
    public static double MIN_TRUSTED_DISTANCE_IN = 40.0;
    public static double MAX_TRUSTED_DISTANCE_IN = 75.0;
    /** Largest |angle| measured is ~47 deg; the angle term stops growing past this. */
    public static double MAX_ANGLE_DEG = 50.0;

    /**
     * Flywheel target for a robot at (x, y) shooting at the hive at (targetX, targetY),
     * ticks/s. Works for any hive. Returns NaN if the pose is invalid. Too close to score
     * returns MAX_VELOCITY; check isTrusted() before relying on it.
     */
    public static double getVelocity(double x, double y, double targetX, double targetY) {
        double d = distance(x, y, targetX, targetY);
        if (!Double.isFinite(d) || d <= 0) return Double.NaN;

        double theta = USE_ANGLE_TERM ? THETA_ANGLE_RAD : THETA_RAD;
        double h = USE_ANGLE_TERM ? H_ANGLE_IN : H_IN;
        double k = USE_ANGLE_TERM ? K_ANGLE : K;

        double cos = Math.cos(theta);
        double den = 2.0 * cos * cos * (d * Math.tan(theta) - h);
        if (den <= 0) return MAX_VELOCITY;   // too close for this launch angle

        double v = Math.sqrt(G_IN_S2 * d * d / den) / k;
        if (USE_ANGLE_TERM) {
            double a = angleDegrees(x, y, targetX, targetY);
            v += C_TICKS_PER_DEG * Math.max(-MAX_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, a));
        }

        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, v));
    }

    /** Straight-line distance between the robot and a hive, inches. */
    public static double distance(double x, double y, double targetX, double targetY) {
        return Math.hypot(targetX - x, targetY - y);
    }

    /** Degrees from straight in front of the hive: + = robot right of it, - = left. */
    public static double angleDegrees(double x, double y, double targetX, double targetY) {
        return Math.toDegrees(Math.atan2(x - targetX, targetY - y));
    }

    /** True when the robot is inside the distance and angle range the model was measured on. */
    public static boolean isTrusted(double x, double y, double targetX, double targetY) {
        double d = distance(x, y, targetX, targetY);
        double a = angleDegrees(x, y, targetX, targetY);
        return d >= MIN_TRUSTED_DISTANCE_IN && d <= MAX_TRUSTED_DISTANCE_IN
                && Math.abs(a) <= MAX_ANGLE_DEG;
    }

    public static String modelName() {
        return USE_ANGLE_TERM ? "distance + angle" : "distance only";
    }
}
