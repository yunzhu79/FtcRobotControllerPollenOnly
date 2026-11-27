package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Shoot-on-the-move (SOTM) for BIOBUZZ: turret aim + flywheel speed while the robot drives.
 * Works for all four CELLs (AutoAim.Target RED_1, RED_2, BLUE_1, BLUE_2); pick one with the
 * constructor or setTarget().
 *
 * Method: "virtual goal" (the same idea 11329, 18221 and 23511 use).
 *   A ball leaves the robot carrying the robot's velocity v. During its flight time T the
 *   robot's motion carries the ball sideways by v*T. So instead of the real CELL we aim at
 *       virtualGoal = goal - v * T
 *   and we set the flywheel from the distance to the virtual goal (AutoDistance table).
 *   T depends on that distance, so we iterate a few times (converges in 2-3 passes).
 *
 * Extras taken from the other teams:
 *   - LATENCY_SEC: predict the shooter position a little ahead to cover sensor + loop lag
 *     (23511 BALL_TRANSFER_TIME, 11329 acceleration lead).
 *   - Turret lead: the required turret rate (bearing rate) times TURRET_LEAD_SEC is added to
 *     the turret target, so the PIDF does not trail a moving target (23511 TURRET_VEL_LAG).
 *     Robot ROTATION is already handled by Turret's kV feedforward.
 *   - Shooter point velocity includes rotation (v + w x r) if the turret is off-center (23511).
 *   - Velocities come from Pinpoint and are low-pass filtered (12527 filters Pinpoint too).
 *   - Ready-to-shoot gate also rejects hard acceleration (11329 far-shot rule).
 *
 * Usage, once per loop:
 *     Hardware.odo.update();
 *     sotm.update();                 // aims the turret   (or sotm.updateNoAim())
 *     launcher.setTargetVelocity(sotm.getShooterVelocity());
 *     launcher.launch(request, sotm.isReadyToShoot());
 *
 * Coordinates, CELL positions and TURRET_ZERO_OFFSET_DEG come from AutoAim, so SOTM with the
 * robot stopped gives exactly the AutoAim answer.
 */
public class SOTM {

    // ---------------- Time of flight (TUNE) ----------------
    /**
     * {distance to (virtual) CELL in inches, ball flight time in seconds}, sorted by distance.
     * Flight time = ball leaves flywheel -> ball reaches the CELL. Measure with phone slow-mo
     * (frames / fps) at the same spots as AutoDistance.SHOT_TABLE.
     *
     * 51.77 in: MEASURED 2026-11-25 from (55, 7, 90) -> RED_1, 750 ticks/s. 9 shots, 30 fps video,
     *           launch time from the audio thump, arrival = ball reaches the CELL's front rim.
     *           Mean 0.435 s, median 0.416 s, range 0.38-0.54 s.
     * 69.19 in: NOT measured yet; scaled from the 51.77 point. Re-measure from (10, 10).
     */
    public static double[][] TOF_TABLE = {
            {51.77, 0.43},
            {69.19, 0.50},
    };
    /** Live trim on the whole TOF table. Balls land in the direction of travel -> raise it. */
    public static double TOF_SCALE = 1.0;
    public static int SOLVER_ITERATIONS = 4;

    // ---------------- Lag compensation (TUNE) ----------------
    /** Sensor + loop + feeder-to-exit lag. Shooter position is predicted this far ahead. */
    public static double LATENCY_SEC = 0.08;
    /** Turret target lead = bearing rate * this. Raise until tracking error ~0 while strafing. */
    public static double TURRET_LEAD_SEC = 0.06;
    /** Extra heading prediction (18221 uses 0.05). Leave 0 while Turret kV handles rotation. */
    public static double HEADING_LEAD_SEC = 0.0;

    // ---------------- Velocity estimate ----------------
    /** true: Pinpoint getVelX/getVelY (field frame). false: differentiate the pose. */
    public static boolean USE_PINPOINT_VELOCITY = true;
    /** 0..1, larger = faster but noisier. */
    public static double VELOCITY_FILTER_ALPHA = 0.5;
    public static double ACCEL_FILTER_ALPHA = 0.3;
    /** Readings faster than this are treated as glitches. */
    public static double MAX_ROBOT_SPEED_IN_PER_SEC = 100.0;
    /** Safety cap on how far the virtual goal may move from the real goal. */
    public static double MAX_LEAD_IN = 40.0;

    // ---------------- Shooter point (offset from Pinpoint pose, robot axes) ----------------
    // Keep 0 unless AutoDistance.SHOT_TABLE is re-measured from the same point.
    public static double SHOOTER_FORWARD_IN = 0.0;
    public static double SHOOTER_LEFT_IN = 0.0;

    // ---------------- Ready-to-shoot gate ----------------
    public static double AIM_TOLERANCE_DEG = 2.5;
    public static double MAX_SHOOT_SPEED_IN_PER_SEC = 50.0;
    public static double MAX_SHOOT_ACCEL_IN_PER_SEC2 = 250.0;
    public static double MAX_SHOOT_TURN_DEG_PER_SEC = 120.0;

    private final GoBildaPinpointDriver odo;
    private final Turret turret;
    private AutoAim.Target target;
    private boolean compensationEnabled = true;

    // Velocity / acceleration state (field frame, in/s and in/s^2)
    private double vx = 0, vy = 0, ax = 0, ay = 0, omega = 0;
    private double lastVx = 0, lastVy = 0;
    private double lastX = Double.NaN, lastY = Double.NaN;
    private long lastTimeNs = 0L;
    private boolean velocityReady = false;

    // Outputs
    private double realDistance = Double.NaN, virtualDistance = Double.NaN;
    private double tof = Double.NaN, shooterVelocity = Double.NaN;
    private double virtualGoalX = Double.NaN, virtualGoalY = Double.NaN;
    private double leadRad = Double.NaN, bearingRate = Double.NaN;
    private double radialSpeed = Double.NaN, tangentialSpeed = Double.NaN;
    private double aimTurretRad = Double.NaN, commandTurretRad = Double.NaN;
    private double aimErrorRad = Double.NaN;
    private boolean geometryValid = false, aimed = false, withinLimits = false;

    public SOTM(AutoAim.Target initialTarget, Turret turret) {
        if (initialTarget == null || turret == null) {
            throw new IllegalArgumentException("Target and turret cannot be null");
        }
        if (Hardware.odo == null) {
            throw new IllegalStateException("Call Hardware.init() before creating SOTM");
        }
        this.odo = Hardware.odo;
        this.turret = turret;
        this.target = initialTarget;
    }

    public void setTarget(AutoAim.Target target) {
        if (target == null) throw new IllegalArgumentException("Target cannot be null");
        this.target = target;
    }
    public AutoAim.Target getTarget() { return target; }

    /** false = behave exactly like AutoAim (no lead), handy for A/B testing. */
    public void setCompensationEnabled(boolean enabled) { compensationEnabled = enabled; }
    public boolean isCompensationEnabled() { return compensationEnabled; }

    /** Clear the velocity filter (call after re-localizing or when Pinpoint was not READY). */
    public void resetVelocity() {
        vx = vy = ax = ay = omega = 0;
        lastVx = lastVy = 0;
        lastX = lastY = Double.NaN;
        lastTimeNs = 0L;
        velocityReady = false;
    }

    /** Aim the turret + compute flywheel speed. Call once per loop AFTER Hardware.odo.update(). */
    public void update() {
        aimed = false;
        withinLimits = false;
        aimTurretRad = commandTurretRad = aimErrorRad = Double.NaN;

        if (!computeSolution()) {
            turret.stop();
            return;
        }

        // Turret angle relative to the robot, using the predicted heading.
        double heading = predictedHeading;
        double relative = wrap(Math.atan2(virtualGoalY - shooterY, virtualGoalX - shooterX)
                - heading - Math.toRadians(AutoAim.TURRET_ZERO_OFFSET_DEG));

        // Lead the turret by the bearing rate so the PIDF is not always one step behind.
        double lead = compensationEnabled ? bearingRate * TURRET_LEAD_SEC : 0.0;

        // Closest equivalent angle inside the mechanical limits (same rule as AutoAim).
        double current = turret.getLastAngle();
        double lower = turret.getLowerLimitRadians();
        double upper = turret.getUpperLimitRadians();
        double best = Double.NaN, bestTravel = Double.POSITIVE_INFINITY;
        for (int k = -3; k <= 3; k++) {
            double candidate = relative + k * 2.0 * Math.PI;
            if (candidate >= lower && candidate <= upper) {
                double travel = Math.abs(candidate - current);
                if (travel < bestTravel) { bestTravel = travel; best = candidate; }
            }
        }
        aimed = true;
        withinLimits = Double.isFinite(best);
        if (!withinLimits) {
            turret.stop();
            return;
        }

        aimTurretRad = best;
        commandTurretRad = best + lead;
        turret.rotateUsingPIDF(commandTurretRad);
        // Judge alignment against the TRUE aim angle, not the led command.
        aimErrorRad = aimTurretRad - turret.getLastAngle();
    }

    /** Flywheel speed + geometry only; the turret is not moved. */
    public void updateNoAim() {
        computeSolution();
        aimed = false;
        withinLimits = false;
        aimTurretRad = commandTurretRad = aimErrorRad = Double.NaN;
    }

    // ---------------------------------------------------------------------------------------
    private double shooterX, shooterY, predictedHeading;

    private boolean computeSolution() {
        geometryValid = false;
        realDistance = virtualDistance = tof = shooterVelocity = Double.NaN;
        virtualGoalX = virtualGoalY = leadRad = bearingRate = Double.NaN;
        radialSpeed = tangentialSpeed = Double.NaN;

        if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            resetVelocity();
            return false;
        }
        Pose2D pose = odo.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double h = wrap(pose.getHeading(AngleUnit.RADIANS));
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h)) {
            resetVelocity();
            return false;
        }
        updateVelocity(x, y);

        double gx, gy;
        switch (target) {
            case RED_1:  gx = AutoAim.RED_1_X_IN;  gy = AutoAim.RED_1_Y_IN;  break;
            case RED_2:  gx = AutoAim.RED_2_X_IN;  gy = AutoAim.RED_2_Y_IN;  break;
            case BLUE_1: gx = AutoAim.BLUE_1_X_IN; gy = AutoAim.BLUE_1_Y_IN; break;
            case BLUE_2: gx = AutoAim.BLUE_2_X_IN; gy = AutoAim.BLUE_2_Y_IN; break;
            default: return false;
        }

        // Shooter point in the field frame and its velocity (v + w x r).
        double c = Math.cos(h), s = Math.sin(h);
        double offX = c * SHOOTER_FORWARD_IN - s * SHOOTER_LEFT_IN;
        double offY = s * SHOOTER_FORWARD_IN + c * SHOOTER_LEFT_IN;
        double useVx = compensationEnabled && velocityReady ? vx - omega * offY : 0.0;
        double useVy = compensationEnabled && velocityReady ? vy + omega * offX : 0.0;
        double useOmega = compensationEnabled && velocityReady ? omega : 0.0;

        double nowX = x + offX, nowY = y + offY;
        realDistance = Math.hypot(gx - nowX, gy - nowY);
        if (!Double.isFinite(realDistance) || realDistance < 1.0) return false;

        // Where the shooter will be when the ball actually leaves (lag compensation).
        double lat = Math.max(0.0, LATENCY_SEC);
        shooterX = nowX + useVx * lat;
        shooterY = nowY + useVy * lat;
        predictedHeading = h + useOmega * (lat + Math.max(0.0, HEADING_LEAD_SEC));

        // Radial (+ = toward goal) and tangential speed, for telemetry and tuning.
        double ux = (gx - nowX) / realDistance, uy = (gy - nowY) / realDistance;
        radialSpeed = useVx * ux + useVy * uy;
        tangentialSpeed = -useVx * uy + useVy * ux;

        // Virtual goal iteration: T = TOF(|goal - v*T - shooter|).
        double vgx = gx, vgy = gy;
        double d = Math.hypot(gx - shooterX, gy - shooterY);
        double t = timeOfFlight(d);
        int iterations = Math.max(1, SOLVER_ITERATIONS);
        for (int i = 0; i < iterations && Double.isFinite(t); i++) {
            double shiftX = -useVx * t, shiftY = -useVy * t;
            double shift = Math.hypot(shiftX, shiftY);
            if (shift > MAX_LEAD_IN) {
                shiftX *= MAX_LEAD_IN / shift;
                shiftY *= MAX_LEAD_IN / shift;
            }
            vgx = gx + shiftX;
            vgy = gy + shiftY;
            d = Math.hypot(vgx - shooterX, vgy - shooterY);
            t = timeOfFlight(d);
        }
        if (!Double.isFinite(t) || !Double.isFinite(d) || d < 1.0) return false;

        virtualGoalX = vgx;
        virtualGoalY = vgy;
        virtualDistance = d;
        tof = t;
        shooterVelocity = AutoDistance.getVelocity(shooterX, shooterY, vgx, vgy);

        double dx = vgx - shooterX, dy = vgy - shooterY;
        leadRad = wrap(Math.atan2(dy, dx) - Math.atan2(gy - nowY, gx - nowX));
        // d(bearing)/dt with the goal fixed and the shooter moving at (useVx, useVy).
        bearingRate = (dy * useVx - dx * useVy) / (d * d);
        geometryValid = true;
        return true;
    }

    private void updateVelocity(double x, double y) {
        long now = System.nanoTime();
        double dt = lastTimeNs == 0L ? 0.0 : (now - lastTimeNs) * 1e-9;
        boolean dtOk = dt >= 0.002 && dt <= 0.25;

        double rawVx, rawVy;
        boolean rawOk;
        if (USE_PINPOINT_VELOCITY) {
            rawVx = odo.getVelX(DistanceUnit.INCH);
            rawVy = odo.getVelY(DistanceUnit.INCH);
            rawOk = Double.isFinite(rawVx) && Double.isFinite(rawVy);
        } else {
            rawOk = dtOk && Double.isFinite(lastX) && Double.isFinite(lastY);
            rawVx = rawOk ? (x - lastX) / dt : 0.0;
            rawVy = rawOk ? (y - lastY) / dt : 0.0;
        }
        double rawOmega = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        if (!Double.isFinite(rawOmega)) rawOmega = 0.0;

        lastX = x;
        lastY = y;
        lastTimeNs = now;

        if (!rawOk || Math.hypot(rawVx, rawVy) > MAX_ROBOT_SPEED_IN_PER_SEC) {
            return;   // glitch: keep the previous filtered value
        }

        double a = clamp(VELOCITY_FILTER_ALPHA, 0.0, 1.0);
        if (!velocityReady) {
            vx = rawVx; vy = rawVy; omega = rawOmega;
            ax = ay = 0;
            velocityReady = true;
        } else {
            vx += a * (rawVx - vx);
            vy += a * (rawVy - vy);
            omega += a * (rawOmega - omega);
            if (dtOk) {
                double b = clamp(ACCEL_FILTER_ALPHA, 0.0, 1.0);
                ax += b * ((vx - lastVx) / dt - ax);
                ay += b * ((vy - lastVy) / dt - ay);
            }
        }
        lastVx = vx;
        lastVy = vy;
    }

    /** Flight time (s) for a distance, linear interpolation, linear extension past the ends. */
    public static double timeOfFlight(double distanceInches) {
        if (!Double.isFinite(distanceInches) || TOF_TABLE.length == 0) return Double.NaN;
        double t;
        if (TOF_TABLE.length == 1) {
            t = TOF_TABLE[0][1];
        } else {
            int last = TOF_TABLE.length - 1;
            int i;
            if (distanceInches <= TOF_TABLE[0][0]) i = 0;
            else if (distanceInches >= TOF_TABLE[last][0]) i = last - 1;
            else { i = 0; while (distanceInches > TOF_TABLE[i + 1][0]) i++; }
            double d1 = TOF_TABLE[i][0], t1 = TOF_TABLE[i][1];
            double d2 = TOF_TABLE[i + 1][0], t2 = TOF_TABLE[i + 1][1];
            t = t1 + (distanceInches - d1) / (d2 - d1) * (t2 - t1);
        }
        return clamp(t * TOF_SCALE, 0.05, 2.0);
    }

    // ---------------- Status ----------------
    public boolean hasValidSolution() { return geometryValid; }
    public boolean isWithinLimits() { return withinLimits; }
    public boolean isVelocityReady() { return velocityReady; }

    public boolean isAtTarget() {
        return aimed && withinLimits && Double.isFinite(aimErrorRad)
                && Math.abs(Math.toDegrees(aimErrorRad)) <= AIM_TOLERANCE_DEG;
    }

    /** Aim is good AND the robot is moving gently enough for the prediction to hold. */
    public boolean isReadyToShoot() {
        return geometryValid && velocityReady && isAtTarget()
                && Double.isFinite(shooterVelocity)
                && getSpeed() <= MAX_SHOOT_SPEED_IN_PER_SEC
                && getAcceleration() <= MAX_SHOOT_ACCEL_IN_PER_SEC2
                && Math.abs(Math.toDegrees(omega)) <= MAX_SHOOT_TURN_DEG_PER_SEC;
    }

    /** Short reason the shot is held, for telemetry. */
    public String getHoldReason() {
        if (!geometryValid) return "no solution (Pinpoint/target)";
        if (!velocityReady) return "velocity filter warming up";
        if (!aimed) return "aim off";
        if (!withinLimits) return "outside turret limits";
        if (!isAtTarget()) return String.format("turret error %.1f deg", getErrorDegrees());
        if (getSpeed() > MAX_SHOOT_SPEED_IN_PER_SEC) return "driving too fast";
        if (getAcceleration() > MAX_SHOOT_ACCEL_IN_PER_SEC2) return "accelerating";
        if (Math.abs(Math.toDegrees(omega)) > MAX_SHOOT_TURN_DEG_PER_SEC) return "turning too fast";
        return "READY";
    }

    // ---------------- Outputs ----------------
    /** Flywheel target (ticks/s) from the VIRTUAL distance, NaN if no solution. */
    public double getShooterVelocity() { return shooterVelocity; }
    public double getDistanceInches() { return realDistance; }
    public double getVirtualDistanceInches() { return virtualDistance; }
    public double getTimeOfFlightSeconds() { return tof; }
    public double getVirtualGoalX() { return virtualGoalX; }
    public double getVirtualGoalY() { return virtualGoalY; }
    public double getLeadDegrees() { return Math.toDegrees(leadRad); }
    public double getBearingRateDegPerSec() { return Math.toDegrees(bearingRate); }
    public double getTargetTurretDegrees() { return Math.toDegrees(aimTurretRad); }
    public double getCommandTurretDegrees() { return Math.toDegrees(commandTurretRad); }
    /** True aim angle minus actual turret angle, degrees. */
    public double getErrorDegrees() { return Math.toDegrees(aimErrorRad); }

    public double getVelocityX() { return vx; }
    public double getVelocityY() { return vy; }
    public double getSpeed() { return Math.hypot(vx, vy); }
    public double getAcceleration() { return Math.hypot(ax, ay); }
    public double getTurnRateDegPerSec() { return Math.toDegrees(omega); }
    /** + = driving toward the CELL. */
    public double getRadialSpeed() { return radialSpeed; }
    /** + = moving to the LEFT as seen from the robot facing the CELL. */
    public double getTangentialSpeed() { return tangentialSpeed; }

    private static double wrap(double r) { return Math.atan2(Math.sin(r), Math.cos(r)); }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
