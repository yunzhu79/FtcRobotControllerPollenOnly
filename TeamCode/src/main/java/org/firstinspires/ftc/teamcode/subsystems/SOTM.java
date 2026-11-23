package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Translational shoot-on-the-move lead for a fixed-elevation launcher.
 * Call Hardware.odo.update() once before update() on each OpMode loop.
 * Coordinates and turret zero must match AutoAim.java.
 */
public class SOTM {
    public static double BALL_HORIZONTAL_SPEED_IN_PER_SEC = 105.0; // TUNE from flight-time tests
    public static double FEEDER_TO_EXIT_SEC = 1.5; // TUNE: feeder activation to first ball exit
    public static double TRACKING_LOOKAHEAD_SEC = 0.0; // TUNE: controller lag, not feeder delay
    public static double VELOCITY_FILTER_ALPHA = 0.35; // 0..1; larger = faster response
    public static double MAX_ROBOT_SPEED_IN_PER_SEC = 80.0; // reject bad pose jumps
    public static double AIM_TOLERANCE_DEG = 3.0;
    // Offset of ball release point from Pinpoint robot center, in robot axes.
    public static double MUZZLE_FORWARD_IN = 0.0;
    public static double MUZZLE_LEFT_IN = 0.0;

    private final GoBildaPinpointDriver odo;
    private final Turret turret;
    private AutoAim.Target target;
    private double lastX = Double.NaN, lastY = Double.NaN;
    private long lastTimeNs = 0;
    private double vx = 0, vy = 0;
    private double distance = Double.NaN, leadDeg = Double.NaN;
    private double targetDeg = Double.NaN, errorDeg = Double.NaN;
    private double flightTime = Double.NaN;
    private boolean valid = false, withinLimits = false, velocityReady = false;

    public SOTM(AutoAim.Target initialTarget, Turret turret) {
        if (initialTarget == null || turret == null || Hardware.odo == null)
            throw new IllegalArgumentException("Initialize Hardware and supply turret/target first");
        this.target = initialTarget;
        this.turret = turret;
        this.odo = Hardware.odo;
    }
    public void setTarget(AutoAim.Target target) {
        if (target == null) throw new IllegalArgumentException("null target");
        this.target = target;
    }
    public AutoAim.Target getTarget() { return target; }
    public void resetVelocity() {
        lastX = lastY = Double.NaN;
        lastTimeNs = 0;
        vx = vy = 0;
        velocityReady = false;
    }
    private double predictionDelay;
    public double getPredictionDelaySeconds() { return predictionDelay; }
    public void update() { update(-1.0); }
    public void update(double feederElapsedSeconds) {
        // Before feeding, aim for a hypothetical release after feeder startup.
        // During continuous feeding, use first-ball countdown, then tracking lag only.
        predictionDelay = TRACKING_LOOKAHEAD_SEC + (feederElapsedSeconds < 0
                ? FEEDER_TO_EXIT_SEC : Math.max(0, FEEDER_TO_EXIT_SEC - feederElapsedSeconds));
        valid = withinLimits = false;
        distance = leadDeg = targetDeg = errorDeg = flightTime = Double.NaN;
        if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            resetVelocity(); turret.stop(); return;
        }
        Pose2D p = odo.getPosition();
        double x = p.getX(DistanceUnit.INCH), y = p.getY(DistanceUnit.INCH);
        double h = p.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h)) {
            resetVelocity(); turret.stop(); return;
        }
        long now = System.nanoTime();
        if (lastTimeNs != 0 && Double.isFinite(lastX) && Double.isFinite(lastY)) {
            double dt = (now - lastTimeNs) * 1e-9;
            if (dt >= 0.005 && dt <= 0.25) {
                double mx = (x-lastX)/dt, my = (y-lastY)/dt;
                if (Math.hypot(mx,my) <= MAX_ROBOT_SPEED_IN_PER_SEC) {
                    double a = Math.max(0, Math.min(1, VELOCITY_FILTER_ALPHA));
                    if (!velocityReady) { vx = mx; vy = my; velocityReady = true; }
                    else { vx += a*(mx-vx); vy += a*(my-vy); }
                } else { velocityReady = false; vx = vy = 0; }
            } else { velocityReady = false; vx = vy = 0; }
        }
        lastX=x; lastY=y; lastTimeNs=now;

        double tx,ty;
        switch(target) {
            case RED_1: tx=AutoAim.RED_1_X_IN; ty=AutoAim.RED_1_Y_IN; break;
            case RED_2: tx=AutoAim.RED_2_X_IN; ty=AutoAim.RED_2_Y_IN; break;
            case BLUE_1: tx=AutoAim.BLUE_1_X_IN; ty=AutoAim.BLUE_1_Y_IN; break;
            case BLUE_2: tx=AutoAim.BLUE_2_X_IN; ty=AutoAim.BLUE_2_Y_IN; break;
            default: turret.stop(); return;
        }
        double c=Math.cos(h), s=Math.sin(h);
        double muzzleX=x+c*MUZZLE_FORWARD_IN-s*MUZZLE_LEFT_IN;
        double muzzleY=y+s*MUZZLE_FORWARD_IN+c*MUZZLE_LEFT_IN;
        double dx=tx-muzzleX, dy=ty-muzzleY;
        distance=Math.hypot(dx,dy);
        double speed=BALL_HORIZONTAL_SPEED_IN_PER_SEC;
        if (!Double.isFinite(distance) || distance<1 || !Double.isFinite(speed) || speed<=0 ||
            !Double.isFinite(predictionDelay) || predictionDelay<0) {
            turret.stop(); return;
        }
        // Robot keeps translating during trigger-to-release delay.
        double rx=dx-vx*predictionDelay, ry=dy-vy*predictionDelay;
        // Solve |r - v*t| = speed*t for positive projectile flight time.
        double a=speed*speed-vx*vx-vy*vy;
        double b=2*(rx*vx+ry*vy);
        double cc=-(rx*rx+ry*ry);
        double disc=b*b-4*a*cc;
        if (a<=0 || disc<0 || !Double.isFinite(disc)) { turret.stop(); return; }
        flightTime=(-b+Math.sqrt(disc))/(2*a);
        if (!Double.isFinite(flightTime) || flightTime<=0) { turret.stop(); return; }
        double compensatedBearing=Math.atan2(ry-vy*flightTime, rx-vx*flightTime);
        double directBearing=Math.atan2(dy,dx);
        leadDeg=Math.toDegrees(wrap(compensatedBearing-directBearing));
        double relative=wrap(compensatedBearing-h-Math.toRadians(AutoAim.TURRET_ZERO_OFFSET_DEG));
        double current=turret.getAngle(), best=Double.NaN, travel=Double.POSITIVE_INFINITY;
        for (int k=-3;k<=3;k++) {
            double candidate=relative+k*2*Math.PI;
            if (candidate>=turret.getLowerLimitRadians() && candidate<=turret.getUpperLimitRadians()
                    && Math.abs(candidate-current)<travel) {
                best=candidate; travel=Math.abs(candidate-current);
            }
        }
        valid=true;
        withinLimits=Double.isFinite(best);
        if (!withinLimits) { turret.stop(); return; }
        turret.rotateUsingPIDF(best);
        targetDeg=Math.toDegrees(best);
        errorDeg=Math.toDegrees(best-turret.getAngle());
    }
    private static double wrap(double a) { return Math.atan2(Math.sin(a),Math.cos(a)); }
    public boolean hasValidTarget() { return valid; }
    public boolean isWithinLimits() { return withinLimits; }
    public boolean isAtTarget() { return velocityReady && valid && withinLimits && Math.abs(errorDeg)<=AIM_TOLERANCE_DEG; }
    public boolean isVelocityReady() { return velocityReady; }
    public double getVelocityX() { return vx; }
    public double getVelocityY() { return vy; }
    public double getSpeed() { return Math.hypot(vx,vy); }
    public double getDistanceInches() { return distance; }
    public double getLeadDegrees() { return leadDeg; }
    public double getTargetTurretDegrees() { return targetDeg; }
    public double getErrorDegrees() { return errorDeg; }
    public double getFlightTimeSeconds() { return flightTime; }
}
