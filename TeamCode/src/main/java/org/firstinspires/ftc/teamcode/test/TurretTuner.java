package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * BIOBUZZ turret tuner. Robot stationary, on blocks or held still.
 * START WITH THE TURRET CENTERED (facing forward); 0 deg = where it is at init.
 * Robot-rotation feedforward (kV) is OFF here; tune kP, kD, kS, ramp first.
 *
 * GAMEPAD 1
 *   D-pad up/down      select parameter (marked with >)
 *   D-pad left/right   decrease / increase selected parameter
 *   A / B              step to +30 / -30 deg
 *   X / Y              step to +90 / -90 deg
 *   Right bumper       step to 0 deg
 *   Left bumper        DISARM (turret power 0)
 *   Right trigger      open-loop ramp + (kS BREAKAWAY TEST: release when it moves)
 *   Left trigger       open-loop ramp - (same, other direction)
 *   Right stick press  toggle flywheel ON/OFF (test with flywheel vibration)
 *
 * After each step the tuner records 3 s and shows overshoot, time to reach
 * within 1 deg, whether it settled within 1 deg, and the final error.
 */
@TeleOp(name = "Turret Tuner", group = "Test")
public class TurretTuner extends OpMode {
    private static final String[] NAMES =
            {"kP", "kD", "kS", "kS ramp deg", "deadband deg", "max power"};
    private static final double[] STEPS = {0.05, 0.005, 0.005, 0.25, 0.05, 0.05};
    private static final double[] MINS = {0.0, 0.0, 0.0, 0.1, 0.0, 0.1};
    private static final double[] MAXS = {5.0, 1.0, 0.5, 10.0, 3.0, 1.0};
    private static final int P = 0, D = 1, S = 2, RAMP = 3, DEAD = 4, MAXP = 5;

    private static final double RECORD_WINDOW_SEC = 3.0;
    private static final double SETTLE_BAND_DEG = 1.0;
    private static final double OPEN_LOOP_RAMP_PER_SEC = 0.05;
    private static final double OPEN_LOOP_LIMIT_DEG = 150.0;
    private static final double MOVE_DETECT_DEG = 0.5;

    private final double[] values = new double[NAMES.length];
    private int selected = 0;

    private Turret turret;
    private Launcher launcher;
    private boolean flywheelOn = false;
    public static double FLYWHEEL_TICKS_PER_SEC = 750.0;
    private double lastTimeSec = -1.0;

    // Closed-loop state
    private boolean armed = false;
    private double targetDeg = 0.0;

    // Step-response recording
    private boolean recording = false;
    private double stepStartDeg, stepStartSec, stepDir;
    private double peakOvershootDeg, tRiseSec, lastOutsideSec, peakVelDegPerSec;
    private boolean hasResult = false;
    private String resultLine1 = "", resultLine2 = "";

    // Open-loop breakaway test state
    private boolean openLoopActive = false;
    private double breakDir = 0.0, ramp = 0.0, breakStartDeg = 0.0;
    private double breakawayPower = Double.NaN, breakawayDeltaDeg = 0.0;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        Hardware.init(hardwareMap);
        turret = new Turret();
        turret.stop();
        launcher = new Launcher();
        launcher.setTargetVelocity(FLYWHEEL_TICKS_PER_SEC);
        launcher.stopLauncher();

        values[P] = turret.getKP();
        values[D] = turret.getKD();
        values[S] = turret.getKS();
        values[RAMP] = 1.0;
        values[DEAD] = 0.3;
        values[MAXP] = 0.6;
        applyGains();
        telemetry.addLine("Turret Tuner ready. Center the turret, then press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = lastTimeSec < 0.0 ? 0.0 : now - lastTimeSec;
        lastTimeSec = now;

        handleParameterKeys();
        if (gamepad1.rightStickButtonWasPressed()) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) launcher.startLauncher(); else launcher.stopLauncher();
        }

        double angleDeg;
        boolean openLoop = runOpenLoop(dt);
        if (openLoop) {
            angleDeg = turret.getAngleDegrees();
        } else {
            handleStepKeys();
            if (armed) {
                turret.rotateUsingPIDF(Math.toRadians(targetDeg));
            } else {
                turret.stop();
            }
            angleDeg = turret.getLastAngleDegrees();
            if (recording) updateRecording(now, angleDeg);
        }
        showTelemetry(angleDeg, openLoop);
    }

    @Override
    public void stop() {
        if (launcher != null) launcher.stopLauncher();
        if (turret != null) turret.stop();
        Hardware.turretServo1.setPower(0.0);
        Hardware.turretServo2.setPower(0.0);
    }

    // ---------------------------------------------------------------- parameters

    private void handleParameterKeys() {
        if (gamepad1.dpadUpWasPressed()) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (gamepad1.dpadDownWasPressed()) selected = (selected + 1) % NAMES.length;
        boolean changed = false;
        if (gamepad1.dpadRightWasPressed()) {
            values[selected] = Math.min(MAXS[selected], values[selected] + STEPS[selected]);
            changed = true;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            values[selected] = Math.max(MINS[selected], values[selected] - STEPS[selected]);
            changed = true;
        }
        if (changed) applyGains();
    }

    private void applyGains() {
        turret.setPIDF(values[P], 0.0, values[D], 0.0);   // kV off: robot is stationary
        turret.setFeedforward(0.0, values[S]);
        turret.setKsRampDegrees(values[RAMP]);
        turret.setDeadbandDegrees(values[DEAD]);
        turret.setMaxPower(values[MAXP]);
    }

    // ---------------------------------------------------------------- steps

    private void handleStepKeys() {
        if (gamepad1.leftBumperWasPressed()) {
            armed = false;
            recording = false;
            turret.stop();
        }
        if (gamepad1.aWasPressed()) commandStep(30.0);
        if (gamepad1.bWasPressed()) commandStep(-30.0);
        if (gamepad1.xWasPressed()) commandStep(90.0);
        if (gamepad1.yWasPressed()) commandStep(-90.0);
        if (gamepad1.rightBumperWasPressed()) commandStep(0.0);
    }

    private void commandStep(double newTargetDeg) {
        double startDeg = turret.getAngleDegrees();
        armed = true;
        targetDeg = newTargetDeg;
        if (Math.abs(newTargetDeg - startDeg) < 2.0) {
            recording = false;   // too small to measure
            return;
        }
        recording = true;
        hasResult = false;
        stepStartDeg = startDeg;
        stepStartSec = getRuntime();
        stepDir = Math.signum(newTargetDeg - startDeg);
        peakOvershootDeg = 0.0;
        tRiseSec = Double.NaN;
        lastOutsideSec = 0.0;
        peakVelDegPerSec = 0.0;
    }

    private void updateRecording(double now, double angleDeg) {
        double t = now - stepStartSec;
        double errDeg = targetDeg - angleDeg;
        peakOvershootDeg = Math.max(peakOvershootDeg, stepDir * (angleDeg - targetDeg));
        if (Double.isNaN(tRiseSec) && Math.abs(errDeg) <= SETTLE_BAND_DEG) tRiseSec = t;
        if (Math.abs(errDeg) > SETTLE_BAND_DEG) lastOutsideSec = t;
        peakVelDegPerSec = Math.max(peakVelDegPerSec, Math.abs(Math.toDegrees(turret.getVelocity())));

        if (t >= RECORD_WINDOW_SEC) {
            recording = false;
            hasResult = true;
            boolean settled = Math.abs(errDeg) <= SETTLE_BAND_DEG;
            resultLine1 = String.format("Step %.0f -> %.0f deg | overshoot %.1f deg | within 1 deg at %s s",
                    stepStartDeg, targetDeg, peakOvershootDeg,
                    Double.isNaN(tRiseSec) ? "never" : String.format("%.2f", tRiseSec));
            resultLine2 = String.format("%s | final error %.2f deg | peak speed %.0f deg/s",
                    settled ? String.format("settled in %.2f s", lastOutsideSec) : "NOT SETTLED",
                    errDeg, peakVelDegPerSec);
        }
    }

    // ---------------------------------------------------------------- open-loop kS test

    /** Returns true while a trigger is held and the open-loop test owns the servos. */
    private boolean runOpenLoop(double dt) {
        double dir = gamepad1.right_trigger > 0.2 ? 1.0 : (gamepad1.left_trigger > 0.2 ? -1.0 : 0.0);
        if (dir == 0.0) {
            if (openLoopActive) {
                openLoopActive = false;
                turret.stop();   // zero power and clear controller history
            }
            return false;
        }
        double angleDeg = turret.getAngleDegrees();
        if (!openLoopActive || dir != breakDir) {
            openLoopActive = true;
            breakDir = dir;
            ramp = 0.0;
            breakStartDeg = angleDeg;
            breakawayPower = Double.NaN;
            breakawayDeltaDeg = 0.0;
            armed = false;
            recording = false;
        }
        boolean atLimit = (dir > 0 && angleDeg > OPEN_LOOP_LIMIT_DEG)
                || (dir < 0 && angleDeg < -OPEN_LOOP_LIMIT_DEG);
        if (atLimit) {
            setServoPower(0.0);
            return true;
        }
        ramp = Math.min(1.0, ramp + OPEN_LOOP_RAMP_PER_SEC * dt);
        setServoPower(dir * ramp);
        if (Double.isNaN(breakawayPower) && Math.abs(angleDeg - breakStartDeg) > MOVE_DETECT_DEG) {
            breakawayPower = ramp;
            breakawayDeltaDeg = angleDeg - breakStartDeg;
        }
        return true;
    }

    private void setServoPower(double power) {
        Hardware.turretServo1.setPower(power);
        Hardware.turretServo2.setPower(power);
    }

    // ---------------------------------------------------------------- telemetry

    private void showTelemetry(double angleDeg, boolean openLoop) {
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.3f", values[i]);
        }
        telemetry.addData("Flywheel (right stick press)", "%s  %.0f ticks/s",
                flywheelOn ? "ON" : "OFF", launcher.getVelocity());
        telemetry.addData("Mode", openLoop ? "OPEN-LOOP kS TEST" : (armed ? "ARMED" : "DISARMED"));
        telemetry.addData("Angle / target / error deg", "%.1f / %.1f / %.2f",
                angleDeg, targetDeg, targetDeg - angleDeg);
        telemetry.addData("Velocity deg/s", "%.1f", Math.toDegrees(turret.getVelocity()));
        telemetry.addData("Servo power", "%.3f", Hardware.turretServo1.getPower());

        if (openLoop || !Double.isNaN(breakawayPower)) {
            telemetry.addData("Ramp power (dir)", "%.3f (%s)", ramp, breakDir > 0 ? "+" : "-");
            if (Double.isNaN(breakawayPower)) {
                telemetry.addLine("Breakaway: not moving yet...");
            } else {
                telemetry.addData("BREAKAWAY power", "%.3f (moved %.1f deg)", breakawayPower, breakawayDeltaDeg);
                if (breakDir * breakawayDeltaDeg < 0) {
                    telemetry.addLine("WRONG DIRECTION: + power moved turret -. Fix servo/encoder sign.");
                }
            }
        }

        if (recording) telemetry.addLine("Recording step...");
        if (hasResult) {
            telemetry.addLine(resultLine1);
            telemetry.addLine(resultLine2);
        }
        telemetry.addLine(String.format("Paste: kP=%.3f kD=%.3f kS=%.3f ramp=%.2f deg dead=%.2f deg maxP=%.2f",
                values[P], values[D], values[S], values[RAMP], values[DEAD], values[MAXP]));
        telemetry.update();
    }
}
