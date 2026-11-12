package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * BIOBUZZ turret: TWO continuous-rotation servos + ONE separate Melonbotics encoder.
 *
 * TurretEncoder.java owns the 60T -> 20T (shared shaft 16T) -> 96T
 * gear conversion. This class works entirely in turret radians.
 *
 * IMPORTANT:
 * - Uses two CRServo devices configured for continuous rotation.
 * - Uses Hardware.turretServo1 and Hardware.turretServo2.
 * - This encoder is connected to the encoder input corresponding to
 *   Hardware.leftFrontDrive; do not reset that motor's encoder counter here.
 * - getAngle() is relative to the quadrature counter's start, not absolute.
 */
public class Turret {
    // Adjust these after checking real physical clearance and cable twist limits.
    // Old code numerically limited motion to +/-90 degrees (despite its +/-45 comment).
    private double lowerLimit = Math.toRadians(-180.0);
    private double upperLimit = Math.toRadians(180.0);
    private static final double AT_LIMIT_TOL = Math.toRadians(2.0);
    private static final double MAX_DEBT = Math.toRadians(90.0);

    private double kP = 0.25;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kV = 0.12;  // robot angular-velocity feedforward (power per rad/s)
    private double kS = 0.1;  // static feedforward power

    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final TurretEncoder turretEncoder;
    private final GoBildaPinpointDriver odo;
    private final PIDFController controller;

    private double angleDebt = 0.0;
    private double previousHeading = Double.NaN;
    private double requestedAngle = 0.0;
    private double lastError = 0.0;

    private double previousVelocityAngle = Double.NaN;
    private long previousVelocityTimeNanos = 0L;
    private double turretVelocity = 0.0;

    public Turret() {
        turretServo1 = Hardware.turretServo1;
        turretServo2 = Hardware.turretServo2;
        turretEncoder = new TurretEncoder();
        odo = Hardware.odo;

        // Feedforward (kV and kS) is calculated in update(); do not double-count
        // it through the FTCLib controller's internal F term.
        controller = new PIDFController(kP, kI, kD, 0.0);
        requestedAngle = getAngle();
    }

    /** Configure actual mechanical safe range, measured in TURRET degrees. */
    public void setLimitsDegrees(double minimum, double maximum) {
        if (!(minimum < maximum)) {
            throw new IllegalArgumentException("Minimum turret angle must be less than maximum");
        }
        lowerLimit = Math.toRadians(minimum);
        upperLimit = Math.toRadians(maximum);
    }

    /** One PIDF tuning interface; f is Pinpoint angular-velocity feedforward kV. */
    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kV = f;  // Retain old public signature: f represents robot-rotation FF
        controller.setPIDF(p, i, d, 0.0);
        resetPIDF();
    }

    public void setFeedforward(double velocityGain, double staticGain) {
        kV = velocityGain;
        kS = staticGain;
    }

    /** Continuous turret angle in radians, including rotations over 360 degrees. */
    public double getAngle() {
        return Math.toRadians(turretEncoder.getPosition());
    }

    /** Continuous turret angle in degrees. */
    public double getAngleDegrees() {
        return turretEncoder.getPosition();
    }

    /** Estimated turret angular speed (rad/s), refreshed when update() runs. */
    public double getVelocity() {
        return turretVelocity;
    }

    /** Robot angular velocity from Pinpoint in rad/s (positive CCW). */
    public double getRobotAngularVelocity() {
        return odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public double getAngleDebt() {
        return angleDebt;
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

    /** Run continuously from the OpMode loop after rotateByAngle(). */
    public void update() {
        final double currentAngle = getAngle();
        updateVelocity(currentAngle);

        // Single PIDF path: compensate for robot heading and track the target.
        double desiredAngle = requestedAngle;
        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        if (!Double.isNaN(previousHeading)) {
            double delta = normalizeRadians(heading - previousHeading);
            desiredAngle -= delta;
        }
        previousHeading = heading;
        desiredAngle += angleDebt;

        double clampedAngle = clamp(desiredAngle, lowerLimit, upperLimit);
        angleDebt = clamp(desiredAngle - clampedAngle, -MAX_DEBT, MAX_DEBT);

        double feedforward = -kV * getRobotAngularVelocity();

        // Explicit measured position and target. Old code did not pass the
        // actual turret angle into the PID controller.
        double pidPower = controller.calculate(currentAngle, clampedAngle);
        lastError = clampedAngle - currentAngle;

        // Static-friction feedforward helps the CR servos overcome deadband.
        // Do not apply it inside the 2-degree tolerance to avoid hunting.
        if (Math.abs(lastError) > Math.toRadians(2.0)) {
            feedforward += Math.signum(turretVelocity) * kS;
        }
        double power = clamp(pidPower + feedforward, -1.0, 1.0);
        if (Math.abs(lastError) <= Math.toRadians(2.0)) {
            power = 0.0;
        }

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

    private void updateVelocity(double angle) {
        long now = System.nanoTime();
        if (!Double.isNaN(previousVelocityAngle) && previousVelocityTimeNanos != 0L) {
            double dt = (now - previousVelocityTimeNanos) * 1e-9;
            if (dt > 0.0) turretVelocity = (angle - previousVelocityAngle) / dt;
        }
        previousVelocityAngle = angle;
        previousVelocityTimeNanos = now;
    }

    private static double clamp(double n, double lo, double hi) {
        return Math.max(lo, Math.min(hi, n));
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /** Reset controller history, not encoder counts or turret hardware zero. */
    public void resetPIDF() {
        controller.reset();
        angleDebt = 0.0;
        previousHeading = Double.NaN;
        lastError = 0.0;
    }

    public void stop() {
        setTurretPower(0.0);
        resetPIDF();
    }

    public boolean atTarget(double toleranceRadians) {
        return Math.abs(lastError) < toleranceRadians;
    }

    public double getError() { return lastError; }
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKV() { return kV; }
    public double getKS() { return kS; }
}
