package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * BIOBUZZ turret: TWO continuous-rotation servos + ONE separate Melonbotics encoder.
 *
 * TurretEncoder.java owns the 60T -> 20T (shared shaft 16T) -> 96T
 * gear conversion. This class works entirely in turret radians.
 *
 * Control law (all angles in radians, all gains are "power" units):
 *   power = kP*error + kI*integral - kD*turretVelocity - kV*robotOmega + kS*ramp*sign(error)
 *
 *  - kP: power per radian of error.
 *  - kD: power per (rad/s) of TURRET velocity, taken on the measurement (damping).
 *        Ratio kD/kP is a time constant; working turrets use about 0.05-0.12 s.
 *  - kV: power per (rad/s) of ROBOT rotation (Pinpoint heading velocity).
 *  - kS: static-friction power, ramped in over ksRampDegrees so it does not
 *        switch on like a bang-bang controller.
 *
 * The PID uses its own measured dt (no FTCLib controller), so units are unambiguous.
 *
 * IMPORTANT:
 * - Uses Hardware.turretServo1 and Hardware.turretServo2.
 * - The encoder is connected to the encoder input of Hardware.leftFrontDrive;
 *   do not reset that motor's encoder counter here.
 * - getAngle() is relative to the quadrature counter's start, not absolute.
 */
public class Turret {
    // Adjust these after checking real physical clearance and cable twist limits.
    private double lowerLimit = Math.toRadians(-180.0);
    private double upperLimit = Math.toRadians(180.0);
    private static final double AT_LIMIT_TOL = Math.toRadians(2.0);

    // ---- Gains (starting point; tune on the robot) ----
    private double kP = 0.4;    // power per rad of error
    private double kI = 0.0;    // power per (rad*s) of accumulated error
    private double kD = 0.04;   // power per (rad/s) of turret velocity (kD/kP = 0.1 s)
    private double kV = 0.035;  // robot angular-velocity feedforward (power per rad/s)
    private double kS = 0.1;    // static feedforward power

    private double ksRampRad = Math.toRadians(5.0);   // kS fades in over this error
    private double deadbandRad = Math.toRadians(0.3); // P/I/kS off inside this error
    private double maxPower = 1.0;

    private static final double INTEGRAL_ZONE_RAD = Math.toRadians(10.0); // only integrate when close
    private static final double INTEGRAL_MAX_POWER = 0.2;                 // cap on kI*integral
    private static final double VELOCITY_FILTER_ALPHA = 0.5;              // 1.0 = no filtering

    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final TurretEncoder turretEncoder;
    private final GoBildaPinpointDriver odo;

    private double requestedAngle = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;

    private double previousAngle = Double.NaN;
    private long lastTimeNanos = 0L;
    private double loopTimeSec = 0.0;
    private double turretVelocity = 0.0;
    private double lastAngle = 0.0;  // angle from the most recent encoder read in update()/stop()

    public Turret() {
        turretServo1 = Hardware.turretServo1;
        turretServo2 = Hardware.turretServo2;
        turretEncoder = new TurretEncoder();
        odo = Hardware.odo;
        requestedAngle = getAngle();
        lastAngle = requestedAngle;
    }

    /** Configure actual mechanical safe range, measured in TURRET degrees. */
    public void setLimitsDegrees(double minimum, double maximum) {
        if (!(minimum < maximum)) {
            throw new IllegalArgumentException("Minimum turret angle must be less than maximum");
        }
        lowerLimit = Math.toRadians(minimum);
        upperLimit = Math.toRadians(maximum);
    }

    /**
     * One PIDF tuning interface. p: power/rad, i: power/(rad*s),
     * d: power per (rad/s) of turret velocity, f: robot-rotation feedforward kV.
     */
    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kV = f;
        resetPIDF();
    }

    public void setFeedforward(double velocityGain, double staticGain) {
        kV = velocityGain;
        kS = staticGain;
    }

    /** kS fades in linearly over this many degrees of error. */
    public void setKsRampDegrees(double degrees) {
        ksRampRad = Math.max(Math.toRadians(0.1), Math.toRadians(degrees));
    }

    /** Inside this error the turret only applies damping and robot-rotation feedforward. */
    public void setDeadbandDegrees(double degrees) {
        deadbandRad = Math.max(0.0, Math.toRadians(degrees));
    }

    public void setMaxPower(double power) {
        maxPower = clamp(Math.abs(power), 0.0, 1.0);
    }

    /** Continuous turret angle in radians, including rotations over 360 degrees. */
    public double getAngle() {
        return Math.toRadians(turretEncoder.getPosition());
    }

    /** Continuous turret angle in degrees. */
    public double getAngleDegrees() {
        return turretEncoder.getPosition();
    }

    /**
     * Turret angle (radians) from the last update()/stop() encoder read. Does NOT touch the
     * hardware, so use it for telemetry and branch selection to keep one encoder read per loop.
     */
    public double getLastAngle() {
        return lastAngle;
    }

    /** Same as getLastAngle() but in degrees. */
    public double getLastAngleDegrees() {
        return Math.toDegrees(lastAngle);
    }

    /** Filtered turret angular speed (rad/s), refreshed when update() runs. */
    public double getVelocity() {
        return turretVelocity;
    }

    /** Time between the last two update() calls, in milliseconds (loop-rate check). */
    public double getLoopTimeMs() {
        return loopTimeSec * 1000.0;
    }

    /** Robot angular velocity from Pinpoint in rad/s (positive CCW). */
    public double getRobotAngularVelocity() {
        return odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    /**
     * Commands an angle relative to the startup encoder reference, in radians.
     * Call this every control loop for a moving target.
     */
    public void rotateUsingPIDF(double targetAngle) {
        requestedAngle = targetAngle;
        update();
    }

    /**
     * Set a NEW relative target, measured in turret degrees.
     * CR servos require update() every OpMode loop for feedback control.
     */
    public void rotateByAngle(double deltaDegrees) {
        requestedAngle = getAngle() + Math.toRadians(deltaDegrees);
        resetPIDF();
    }

    /** Run continuously from the OpMode loop. */
    public void update() {
        final double currentAngle = getAngle();
        lastAngle = currentAngle;
        final double dt = updateTimingAndVelocity(currentAngle);

        double clampedTarget = clamp(requestedAngle, lowerLimit, upperLimit);
        double error = clampedTarget - currentAngle;
        lastError = error;

        // Damping on the measurement: no derivative kick when the target jumps.
        double damping = -kD * turretVelocity;

        // Counter-rotate against robot rotation so the turret holds its field aim.
        double robotRotationFF = -kV * getRobotAngularVelocity();

        double power;
        if (Math.abs(error) <= deadbandRad) {
            // On target: do not push, but keep tracking robot rotation and keep damping.
            integral = 0.0;
            power = damping + robotRotationFF;
        } else {
            double pTerm = kP * error;

            if (kI != 0.0 && dt > 0.0 && Math.abs(error) < INTEGRAL_ZONE_RAD) {
                integral += error * dt;
                double integralLimit = INTEGRAL_MAX_POWER / Math.abs(kI);
                integral = clamp(integral, -integralLimit, integralLimit);
            } else if (Math.abs(error) >= INTEGRAL_ZONE_RAD) {
                integral = 0.0;
            }
            double iTerm = kI * integral;

            // Static friction ramps in with error instead of switching on at a threshold.
            double ramp = Math.min(1.0, Math.abs(error) / ksRampRad);
            double staticTerm = Math.signum(error) * kS * ramp;

            power = pTerm + iTerm + damping + robotRotationFF + staticTerm;
        }
        power = clamp(power, -maxPower, maxPower);

        // Never drive farther beyond the configured safe turret range.
        if (currentAngle <= lowerLimit + AT_LIMIT_TOL && power < 0.0) {
            power = 0.0;
        }
        if (currentAngle >= upperLimit - AT_LIMIT_TOL && power > 0.0) {
            power = 0.0;
        }
        setTurretPower(power);
    }

    private void setTurretPower(double power) {
        // Servo directions are configured in Hardware.init().
        turretServo1.setPower(power);
        turretServo2.setPower(power);
    }

    /** Updates loop time and filtered turret velocity. Returns dt in seconds (0 if invalid). */
    private double updateTimingAndVelocity(double angle) {
        long now = System.nanoTime();
        double dt = 0.0;
        if (lastTimeNanos != 0L) {
            dt = (now - lastTimeNanos) * 1e-9;
        }
        lastTimeNanos = now;

        boolean dtValid = dt > 0.0005 && dt < 0.25;
        loopTimeSec = dtValid ? dt : loopTimeSec;

        if (dtValid && !Double.isNaN(previousAngle)) {
            double rawVelocity = (angle - previousAngle) / dt;
            turretVelocity += VELOCITY_FILTER_ALPHA * (rawVelocity - turretVelocity);
        }
        previousAngle = angle;
        return dtValid ? dt : 0.0;
    }

    private static double clamp(double n, double lo, double hi) {
        return Math.max(lo, Math.min(hi, n));
    }

    /** Reset controller history, not encoder counts or turret hardware zero. */
    public void resetPIDF() {
        integral = 0.0;
        lastError = 0.0;
        turretVelocity = 0.0;
        previousAngle = Double.NaN;
        lastTimeNanos = 0L;
    }

    public void stop() {
        setTurretPower(0.0);
        resetPIDF();
        lastAngle = getAngle();
    }

    public boolean atTarget(double toleranceRadians) {
        return Math.abs(lastError) < toleranceRadians;
    }

    public double getLowerLimitRadians() { return lowerLimit; }
    public double getUpperLimitRadians() { return upperLimit; }
    public double getError() { return lastError; }
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKV() { return kV; }
    public double getKS() { return kS; }
}
