package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel launcher.
 *
 * Speed control: ONE software controller reads ONE encoder (launcher1) and sends the SAME
 * power to both motors, so the two motors never fight each other. Output is
 *
 *     power = (kS + kV * target) * (NOMINAL_VOLTAGE / batteryVoltage) + kP * error
 *
 * kV does most of the work (it is the power needed to hold a speed); kP only cleans up the
 * remaining error. update() MUST be called once per loop. launch() calls it for you.
 *
 * Feeding (no gate, all balls in one burst): like 23511's ClearLaunch, the feeder AND intake
 * run only while the flywheel is near speed. If the flywheel drops more than PAUSE_DROP, both
 * stop until it recovers. The burst ends after the feeder has run FEED_TIME_SECONDS in total.
 */
public class Launcher {
    public enum LaunchState {
        IDLE,
        SPIN_UP,
        WAIT_FOR_STOPPER,
        LAUNCH
    }

    // ---------------- Flywheel controller gains (tune with FlywheelTuner) ----------------
    /** Power per (tick/s) at NOMINAL_VOLTAGE. Tune this FIRST with kP = 0. */
    public static double kV = 0.00059;   // tuned 2026-11-25 at 700 ticks/s, 12.4 V
    /** Static friction power. Leave at 0 unless the flywheel can't hold low speeds. */
    public static double kS = 0.0;
    /** Power per (tick/s) of error. Tune SECOND, after kV is right. */
    public static double kP = 0.006;     // tuned: recovers in ~0.01 s, no oscillation
    public static final double NOMINAL_VOLTAGE = 12.0;

    // ---------------- Shooting ----------------
    private final double STOPPER_OPEN_DELAY_SECONDS = 0.5;
    private final double STOP_SPEED = 0.0;
    private final double HALF_SPEED = 0.8;
    private final double FULL_SPEED = 1.0;
    /** Hub velocity reads come in steps of 20 ticks/s, so 30 means "reading within one step". */
    public static double VELOCITY_TOLERANCE = 30.0;

    // ---------------- Gateless burst feeding (23511 ClearLaunch style) ----------------
    /** Feeder power while feeding. Lower = balls more spaced out = more recovery time between them. */
    public static double FEED_POWER = 1.0;
    /** Intake power while feeding (pushes balls toward the feeder). */
    public static double INTAKE_FEED_POWER = 0.8;
    /** Pause feeder AND intake when the flywheel is this far below target (ticks/s). */
    public static double PAUSE_DROP = 40.0;
    /** Total time the feeder must actually run to empty the magazine (pauses don't count). */
    public static double FEED_TIME_SECONDS = 1.2;
    /** Hard limit for the whole burst, pauses included. */
    public static double BURST_TIMEOUT_SECONDS = 3.0;

    private boolean feedPaused = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double feedTimeAccumulated = 0;

    // Diagnostics for the last burst.
    private int pauseCount = 0;
    /** Approximate ball count from speed dips. Telemetry/logging ONLY; it does not control feeding. */
    public static double SHOT_COUNT_DROP = 50.0;
    public static double SHOT_COUNT_REARM = 30.0;
    private int shotsFired = 0;
    private boolean shotArmed = true;
    private double maxDipThisBurst = 0;
    private double burstDuration = 0;
    private String lastEndReason = "-";

    private double targetVelocity = 750;   // ticks/s requested by startLauncher()
    private double activeTarget = 0;       // ticks/s the controller is holding right now (0 = off)

    private static LaunchState launchState;
    private final ElapsedTime launchTimer = new ElapsedTime();

    // Battery voltage is a slow hub read, so only refresh it a few times per second.
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_REFRESH_SECONDS = 0.25;
    private double batteryVoltage = NOMINAL_VOLTAGE;

    private double lastVelocity = 0;
    private double lastPower = 0;

    private static DcMotorEx launcher1 = null;
    private static DcMotorEx launcher2 = null;
    private static LaunchStopper launchStopper = null;
    private static DcMotorEx feeder = null;
    private static DcMotorEx intake = null;

    public Launcher() {
        launcher1 = Hardware.launcher1;
        launcher2 = Hardware.launcher2;
        launchStopper = new LaunchStopper();
        feeder = Hardware.feeder;
        intake = Hardware.intake;
        launchState = LaunchState.IDLE;
        readVoltage();
        setMotorPower(0);
    }

    // ======================= Speed control =======================

    /** Run the flywheel controller. Call exactly once per loop (launch() already does). */
    public void update() {
        if (voltageTimer.seconds() >= VOLTAGE_REFRESH_SECONDS) {
            readVoltage();
        }

        lastVelocity = launcher1.getVelocity();

        if (activeTarget <= 0) {
            setMotorPower(0);
            return;
        }

        double error = activeTarget - lastVelocity;
        double feedforward = (kS + kV * activeTarget) * (NOMINAL_VOLTAGE / batteryVoltage);
        double power = feedforward + kP * error;

        // Never drive the flywheel backwards; with FLOAT it simply coasts down when too fast.
        setMotorPower(Range.clip(power, 0.0, 1.0));
    }

    private void readVoltage() {
        double v = Hardware.voltageSensor.getVoltage();
        if (v > 6.0) batteryVoltage = v;   // ignore bad readings
        voltageTimer.reset();
    }

    private void setMotorPower(double power) {
        launcher1.setPower(power);
        launcher2.setPower(power);
        lastPower = power;
    }

    public void startLauncher() {
        activeTarget = targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        activeTarget = targetVelocity;
    }

    public void stopLauncher() {
        cancelShot();
        activeTarget = 0;
        setMotorPower(0);
        launchStopper.setClose();
        launchState = LaunchState.IDLE;
    }

    /** Flywheel speed from the single control encoder (launcher1), ticks/s. */
    public double getVelocity() {
        return lastVelocity;
    }

    /** Raw velocity of each motor, for telemetry only. */
    public double getVelocity1() { return launcher1.getVelocity(); }
    public double getVelocity2() { return launcher2.getVelocity(); }

    public double getPower() { return lastPower; }
    public double getBatteryVoltage() { return batteryVoltage; }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isAtTargetVelocity() {
        return activeTarget > 0 && Math.abs(lastVelocity - activeTarget) < VELOCITY_TOLERANCE;
    }

    // ======================= Shooting =======================

    public void cancelShot() {
        feeder.setPower(STOP_SPEED);
        launchStopper.setClose();
        intake.setPower(STOP_SPEED);
        launchState = LaunchState.IDLE;
        launchTimer.reset();
    }

    public double getFeederElapsedSeconds() {
        return launchState == LaunchState.LAUNCH ? launchTimer.seconds() : -1.0;
    }

    public LaunchState getLaunchState() {
        return launchState;
    }

    public void launch(boolean shotRequested) {
        launchInternal(shotRequested, true, false);
    }

    public void launch(boolean shotRequested, boolean aimReady) {
        launchInternal(shotRequested, aimReady, true);
    }

    private void launchInternal(boolean shotRequested, boolean aimReady, boolean gated) {
        update();

        if (launchState == LaunchState.IDLE && shotRequested) {
            feeder.setPower(STOP_SPEED);
            intake.setPower(HALF_SPEED);
            launchStopper.setClose();
            activeTarget = targetVelocity;
            launchState = LaunchState.SPIN_UP;
            launchTimer.reset();
        }

        if (launchState == LaunchState.SPIN_UP) {
            if (isAtTargetVelocity() && aimReady) {
                // Open the stopper first, but do not feed any balls yet.
                launchStopper.setOpen();
                launchState = LaunchState.WAIT_FOR_STOPPER;
                launchTimer.reset();
            }
        } else if (launchState == LaunchState.WAIT_FOR_STOPPER) {
            if (gated && (!aimReady || !isAtTargetVelocity())) {
                launchStopper.setClose();
                launchState = LaunchState.SPIN_UP;
                launchTimer.reset();
            } else if (launchTimer.seconds() >= STOPPER_OPEN_DELAY_SECONDS) {
                feedTimeAccumulated = 0;
                pauseCount = 0;
                shotsFired = 0;
                shotArmed = true;
                maxDipThisBurst = 0;
                lastEndReason = "running";
                setFeeding(true);
                loopTimer.reset();
                launchState = LaunchState.LAUNCH;
                launchTimer.reset();
            }
        } else if (launchState == LaunchState.LAUNCH) {
            updateFeeding();
        }
    }

    private void setFeeding(boolean on) {
        feeder.setPower(on ? FEED_POWER : STOP_SPEED);
        intake.setPower(on ? INTAKE_FEED_POWER : STOP_SPEED);
        feedPaused = !on;
    }

    /**
     * Gateless burst: run feeder + intake while the flywheel is near speed; pause BOTH when it
     * drops more than PAUSE_DROP; resume once it is back within VELOCITY_TOLERANCE. The burst ends
     * after the feeder has actually run for FEED_TIME_SECONDS. No ball counting needed.
     */
    private void updateFeeding() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double dip = activeTarget - lastVelocity;
        maxDipThisBurst = Math.max(maxDipThisBurst, dip);

        // Approximate shot count (logging only): count each dip once, re-arm after recovery.
        if (shotArmed && dip >= SHOT_COUNT_DROP) {
            shotsFired++;
            shotArmed = false;
        } else if (!shotArmed && dip < SHOT_COUNT_REARM) {
            shotArmed = true;
        }

        if (!feedPaused) {
            feedTimeAccumulated += dt;
            if (dip >= PAUSE_DROP) {
                setFeeding(false);
                pauseCount++;
            }
        } else if (isAtTargetVelocity()) {
            setFeeding(true);
        }

        boolean done = false;
        if (feedTimeAccumulated >= FEED_TIME_SECONDS) {
            done = true;
            lastEndReason = "feed time done";
        } else if (launchTimer.seconds() >= BURST_TIMEOUT_SECONDS) {
            done = true;
            lastEndReason = "burst timeout";
        }

        if (done) {
            burstDuration = launchTimer.seconds();
            feeder.setPower(STOP_SPEED);
            launchStopper.setClose();
            intake.setPower(FULL_SPEED);
            feedPaused = false;
            launchState = LaunchState.IDLE;
            launchTimer.reset();
        }
    }

    /**
     * Approximate number of balls fired in the current/last burst, from speed dips.
     * Two balls that hit the flywheel back-to-back can count as one. Use for logging only.
     */
    public int getShotsFired() { return shotsFired; }

    /** Why the last burst ended: "feed time done" or "burst timeout". */
    public String getLastEndReason() { return lastEndReason; }

    /** Biggest speed drop (ticks/s) seen during the last burst. */
    public double getMaxDip() { return maxDipThisBurst; }

    /** How many times feeding paused to let the flywheel recover in the last burst. */
    public int getPauseCount() { return pauseCount; }

    /** Total length of the last burst in seconds, pauses included. */
    public double getBurstDuration() { return burstDuration; }
}
