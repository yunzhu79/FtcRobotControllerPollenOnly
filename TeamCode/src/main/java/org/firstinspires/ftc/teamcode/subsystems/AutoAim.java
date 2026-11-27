package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Pinpoint-only turret aiming + auto distancing for BIOBUZZ.
 *
 * Field frame: Pedro Pathing, origin at bottom-left, +X right, +Y up.
 * Pinpoint is initialized directly in Pedro field coordinates (no conversion).
 * All target positions are PROVISIONAL estimates, not rulebook-derived field coordinates.
 * The HIVE elevation drawing does not determine their horizontal field positions.
 * The OpMode must call Hardware.odo.update() before calling update() / updateDistance().
 * Does not command the launcher or feed balls; read getShooterVelocity() and pass it to
 * Launcher.setTargetVelocity(). Does not compensate for ball flight motion.
 * Requires the updated Turret.java that accepts absolute robot-relative angles.
 * Reads the turret encoder once per loop (inside turret.update()).
 *
 * Auto distancing: getShooterVelocity() = AutoDistance.getVelocity(robot pose, selected
 * target). The model was fitted against RED_1 and is applied to every CELL as the same
 * surface shifted to that CELL's (x, y).
 */
public class AutoAim {
    public enum Target { RED_1, RED_2, BLUE_1, BLUE_2 }

    // PROVISIONAL horizontal aiming coordinates (inches).
    // The provided HIVE/CELL figures give heights and 25.5 in center spacing,
    // but do not fix these coordinates relative to field boundaries.
    // Measure field locations before using these values to score.
    public static double RED_1_X_IN = 59.25;
    public static double RED_1_Y_IN = 58.6;
    public static double RED_2_X_IN = 59.25;
    public static double RED_2_Y_IN = 85.4;
    public static double BLUE_1_X_IN = 84.75;
    public static double BLUE_1_Y_IN = 85.4;
    public static double BLUE_2_X_IN = 84.75;
    public static double BLUE_2_Y_IN = 58.6;

    // Bearing of the turret's firing axis, relative to robot +X, at encoder zero.
    public static double TURRET_ZERO_OFFSET_DEG = 0.0;
    public static double AIM_TOLERANCE_DEG = 3.0;

    private final GoBildaPinpointDriver odo;
    private final Turret turret;
    private Target target = Target.BLUE_1;

    private double distanceInches = Double.NaN;
    private double bearingRadians = Double.NaN;
    private double headingRadians = Double.NaN;
    private double targetTurretRadians = Double.NaN;
    private double errorRadians = Double.NaN;
    private double shooterVelocity = Double.NaN;
    private double angleDegrees = Double.NaN;
    private boolean distanceTrusted = false;
    private boolean validTarget = false;
    private boolean withinLimits = false;

    public AutoAim(Turret turret) {
        this(Target.BLUE_1, turret);
    }

    public AutoAim(Target initialTarget, Turret turret) {
        if (initialTarget == null || turret == null) {
            throw new IllegalArgumentException("Target and turret cannot be null");
        }
        this.odo = Hardware.odo;
        if (this.odo == null) {
            throw new IllegalStateException("Call Hardware.init() before creating AutoAim");
        }
        this.turret = turret;
        this.target = initialTarget;
    }

    public void setTarget(Target target) {
        if (target == null) throw new IllegalArgumentException("Target cannot be null");
        this.target = target;
    }

    public Target getTarget() { return target; }

    /**
     * Distance + flywheel speed only; does NOT move the turret.
     * Use this when aiming is off. Call once per loop AFTER Hardware.odo.update().
     */
    public void updateDistance() {
        computeGeometry();
        validTarget = false;
        withinLimits = false;
        targetTurretRadians = Double.NaN;
        errorRadians = Double.NaN;
    }

    /** Aim the turret + distance + flywheel speed. Call once per loop AFTER Hardware.odo.update(). */
    public void update() {
        validTarget = false;
        withinLimits = false;
        targetTurretRadians = Double.NaN;
        errorRadians = Double.NaN;

        if (!computeGeometry() || distanceInches < 1.0) {
            turret.stop();
            return;
        }

        double relativeAngle = wrap(bearingRadians - headingRadians
                - Math.toRadians(TURRET_ZERO_OFFSET_DEG));

        // Choose the closest equivalent angle that lies within mechanical limits.
        double current = turret.getLastAngle();  // no extra encoder read
        double lower = turret.getLowerLimitRadians();
        double upper = turret.getUpperLimitRadians();
        double best = Double.NaN;
        double bestTravel = Double.POSITIVE_INFINITY;
        for (int k = -3; k <= 3; k++) {
            double candidate = relativeAngle + k * 2.0 * Math.PI;
            if (candidate >= lower && candidate <= upper) {
                double travel = Math.abs(candidate - current);
                if (travel < bestTravel) {
                    bestTravel = travel;
                    best = candidate;
                }
            }
        }

        validTarget = true;
        withinLimits = Double.isFinite(best);
        if (!withinLimits) {
            turret.stop();
            return;
        }

        targetTurretRadians = best;
        turret.rotateUsingPIDF(targetTurretRadians);
        errorRadians = turret.getError();  // target minus the angle used by update()
    }

    /**
     * Reads the pose and fills distance, bearing, heading and shooter velocity.
     * Returns false (all NaN) if the pose or target is not usable.
     */
    private boolean computeGeometry() {
        distanceInches = Double.NaN;
        bearingRadians = Double.NaN;
        headingRadians = Double.NaN;
        shooterVelocity = Double.NaN;
        angleDegrees = Double.NaN;
        distanceTrusted = false;

        double tx, ty;
        switch (target) {
            case RED_1:  tx = RED_1_X_IN;  ty = RED_1_Y_IN;  break;
            case RED_2:  tx = RED_2_X_IN;  ty = RED_2_Y_IN;  break;
            case BLUE_1: tx = BLUE_1_X_IN; ty = BLUE_1_Y_IN; break;
            case BLUE_2: tx = BLUE_2_X_IN; ty = BLUE_2_Y_IN; break;
            default: throw new IllegalStateException("Unknown target");
        }

        Pose2D pose = odo.getPosition();
        // Pinpoint pose is already expressed in Pedro field coordinates.
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = wrap(pose.getHeading(AngleUnit.RADIANS));
        if (!Double.isFinite(tx) || !Double.isFinite(ty) ||
                !Double.isFinite(x) || !Double.isFinite(y) ||
                !Double.isFinite(heading)) {
            return false;
        }

        double dx = tx - x;
        double dy = ty - y;
        distanceInches = Math.hypot(dx, dy);
        bearingRadians = Math.atan2(dy, dx);
        headingRadians = heading;
        angleDegrees = AutoDistance.angleDegrees(x, y, tx, ty);
        shooterVelocity = AutoDistance.getVelocity(x, y, tx, ty);
        distanceTrusted = AutoDistance.isTrusted(x, y, tx, ty);
        return true;
    }

    public double getDistanceInches() { return distanceInches; }
    /** Flywheel target (ticks/s) for the current distance, or NaN if not available. */
    public double getShooterVelocity() { return shooterVelocity; }
    /** Degrees from straight in front of the target: + = robot right of it, - = left. */
    public double getShotAngleDegrees() { return angleDegrees; }
    /** True when distance and angle are inside the range the auto-distance model was measured on. */
    public boolean isDistanceTrusted() { return distanceTrusted; }
    public double getBearingDegrees() { return Math.toDegrees(bearingRadians); }
    public double getTargetTurretDegrees() { return Math.toDegrees(targetTurretRadians); }
    public double getErrorDegrees() { return Math.toDegrees(errorRadians); }
    public boolean hasValidTarget() { return validTarget; }
    public boolean isWithinLimits() { return withinLimits; }
    public boolean isAtTarget() {
        return validTarget && withinLimits &&
                Math.abs(getErrorDegrees()) <= AIM_TOLERANCE_DEG;
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
