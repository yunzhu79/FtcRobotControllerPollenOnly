package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RGBIndicator;
import org.firstinspires.ftc.teamcode.subsystems.SOTM;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Shoot-on-the-move test, RED HIVE CELL 1 only: SOTM turret aim + SOTM flywheel speed + gated shooting.
 *
 * Start pose: (55, 7) facing 90 deg, Pedro field coordinates. Turret at 0 deg at INIT.
 *
 * Controls (gamepad1):
 *   Sticks             drive (scaled by DRIVE_SCALE)
 *   Right bumper       arm a shot: the burst starts as soon as SOTM says READY, then runs
 *                      the same launcher sequence as teleop.java (set REQUIRE_SOTM_READY =
 *                      false to fire immediately like teleop)
 *   B                  cancel shot / disarm
 *   Intake             always on (the launcher runs it at 0.8 while feeding)
 *   A                  toggle turret aim
 *   X                  toggle compensation ON (SOTM) / OFF (plain AutoAim) for A/B tests
 *   D-pad up/down      TOF_SCALE  +/- 0.05
 *   D-pad right/left   TURRET_LEAD_SEC +/- 0.01
 *
 * RAIL MODE (the robot holds its row and heading for you; it never turns):
 *   Y                  toggle AUTO RAIL: strafes back and forth between RAIL_X_MIN_IN and
 *                      RAIL_X_MAX_IN at RAIL_AUTO_SPEED, holding Y = RAIL_Y_IN, heading 90
 *   Hold LB            MANUAL RAIL: left stick X = speed along the row (right = +X),
 *                      Y and heading held, slows down by itself before the X limits
 *   B                  stops AUTO RAIL (and cancels a shot)
 *   On entry the robot eases from Y = 7 to RAIL_Y_IN (12) at RAIL_Y_APPROACH_SPEED.
 *   It ABORTS (motors 0) if Y, heading or X tracking goes wrong, e.g. a reversed motor
 *   direction, so it does not drive into the wall. Release LB / press Y again to retry.
 *
 * Tuning order (copy final values into SOTM.java):
 *  1. Velocity sign: drive toward +X field. "Robot vx" must be positive. If not, set
 *     SOTM.USE_PINPOINT_VELOCITY = false.
 *  2. Robot stopped: lead must read ~0 deg and shots must match AutoAimTest.
 *  3. TURRET_LEAD_SEC: strafe slowly past the CELL (no shooting). "Aim error" should
 *     average ~0. If it stays on the same side as "Bearing rate", raise the lead.
 *  4. TOF_SCALE: strafe at a steady medium speed and shoot.
 *       balls land toward the direction you are driving -> TOF_SCALE up
 *       balls land behind (opposite your direction)     -> TOF_SCALE down
 *     Then drive straight toward / away from the CELL:
 *       long while driving toward, or short while driving away -> TOF_SCALE up
 *  5. When it is right at several distances, measure real flight times and fill
 *     SOTM.TOF_TABLE, then set TOF_SCALE back to 1.0.
 */
@TeleOp(name = "SOTM Test", group = "Test")
public class SOTMTest extends OpMode {
    public static double START_X_IN = 55.0;
    public static double START_Y_IN = 7.0;
    public static double START_HEADING_DEG = 90.0;

    /** Used until the first valid SOTM solution. */
    public static double FALLBACK_LAUNCHER_TICKS_PER_SEC = 750.0;
    /** Flywheel target slew limit. Keeps a moving target from looking like a shot dip. */
    public static double FLYWHEEL_SLEW_TICKS_PER_SEC2 = 300.0;
    /** Lower than 1 gives smoother, repeatable test passes. */
    public static double DRIVE_SCALE = 0.7;

    /** true: RB waits until SOTM is ready. false: RB fires right away, exactly like teleop. */
    public static boolean REQUIRE_SOTM_READY = true;

    // ---- Rail mode (field inches, Pedro coordinates). CHECK the X limits on your field. ----
    public static double RAIL_Y_IN = 12.0;
    public static double RAIL_X_MIN_IN = 30.0;
    public static double RAIL_X_MAX_IN = 80.0;
    public static double RAIL_HEADING_DEG = 90.0;
    public static double RAIL_AUTO_SPEED_IN_PER_SEC = 30.0;
    public static double RAIL_MANUAL_MAX_SPEED_IN_PER_SEC = 45.0;
    public static double RAIL_ACCEL_IN_PER_SEC2 = 60.0;
    public static double RAIL_Y_APPROACH_SPEED_IN_PER_SEC = 6.0;
    /** Drive power per in/s (feedforward). ~1 / (robot top speed in in/s). */
    public static double RAIL_KV = 1.0 / 60.0;
    public static double RAIL_KP_X = 0.05;          // power per inch
    public static double RAIL_KP_Y = 0.06;          // power per inch
    public static double RAIL_KP_HEADING = 2.5;     // power per radian (10 deg -> 0.44)
    public static double RAIL_MAX_POWER = 0.8;
    public static double RAIL_MAX_TURN_POWER = 0.4;
    /** Minimum drive power once the robot must move; below this the wheels just stall. */
    public static double RAIL_KS_DRIVE = 0.10;
    /** Minimum turn power once heading error is above RAIL_HEADING_DEADBAND_DEG. */
    public static double RAIL_KS_TURN = 0.08;
    /** No kS when position error is this small (avoids buzzing while holding still). */
    public static double RAIL_POSITION_DEADBAND_IN = 0.4;
    public static double RAIL_HEADING_DEADBAND_DEG = 0.7;
    // Safety: rail mode stops the motors if any of these is exceeded.
    public static double RAIL_ABORT_X_ERROR_IN = 10.0;
    public static double RAIL_ABORT_Y_ERROR_IN = 4.0;
    public static double RAIL_ABORT_HEADING_ERROR_DEG = 10.0;
    /** Never let the robot center get closer than this to the back wall (Y = 0). */
    public static double RAIL_MIN_Y_IN = 5.0;

    // Same turret feedforward as AutoDistanceTest (kV cancels robot rotation, kS friction).
    public static double TURRET_KV = 0.12;
    public static double TURRET_KS = 0.10;

    private GoBildaPinpointDriver odo;
    private Drivetrain drive;
    private Turret turret;
    private Launcher launcher;
    private InTake intake;
    private FlowerIntake flower;
    private RGBIndicator rgb;
    private SOTM sotm;

    private boolean poseReady = false;
    private boolean aiming = false;
    private boolean shotArmed = false;
    // Rail state
    private boolean autoRail = false;
    private boolean railWasActive = false;
    private boolean railLockedOut = false;   // after an abort, until LB released / Y pressed
    private double railSetX = 0, railSetV = 0, railSetY = 0;
    private int railDir = 1;
    private String railStatus = "off";
    private boolean greenOn = false;
    private double flywheelTarget = FALLBACK_LAUNCHER_TICKS_PER_SEC;
    private long lastLoopNs = 0L;
    private double loopMs = 0.0;

    // Snapshot of the conditions when the last ball was detected leaving.
    private int lastShotCount = 0;
    private int totalShots = 0;
    private String lastShot = "none yet";

    @Override public void init() {
        telemetry.setMsTransmissionInterval(50);
        Hardware.init(hardwareMap);
        rgb = new RGBIndicator();
        rgb.setRed();
        odo = Hardware.odo;
        odo.resetPosAndIMU();
        try {
            Thread.sleep(1200);   // let the Pinpoint IMU finish resetting
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }
        odo.update();
        if (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN,
                    AngleUnit.DEGREES, START_HEADING_DEG));
            odo.update();
            poseReady = true;
        }

        drive = new Drivetrain();
        turret = new Turret();
        turret.setFeedforward(TURRET_KV, TURRET_KS);
        turret.stop();
        launcher = new Launcher();
        intake = new InTake();
        flower = new FlowerIntake();
        flower.setUp();
        new LaunchStopper().setClose();
        sotm = new SOTM(AutoAim.Target.RED_1, turret);   // this test: RED HIVE CELL 1 only

        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Pose assigned", poseReady);
        telemetry.addData("Turret deg (should be ~0)", "%.1f", turret.getAngleDegrees());
        telemetry.addLine("Target: RED HIVE CELL 1");
        telemetry.addLine("RB shoot | B cancel | A aim | X comp on/off");
        telemetry.addLine("D-pad: up/down TOF scale, right/left turret lead");
        if (!poseReady) telemetry.addLine("Pinpoint not READY: STOP and re-INIT");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (flower != null) flower.setUp();
    }

    @Override public void start() {
        if (intake != null) intake.startIntake();   // intake always on
        if (!poseReady) return;
        flower.setUp();
        flywheelTarget = FALLBACK_LAUNCHER_TICKS_PER_SEC;
        launcher.setTargetVelocity(flywheelTarget);
        launcher.startLauncher();
        sotm.resetVelocity();
        aiming = true;
    }

    @Override public void loop() {
        if (flower == null) return;
        long now = System.nanoTime();
        double dt = lastLoopNs == 0L ? 0.0 : (now - lastLoopNs) * 1e-9;
        lastLoopNs = now;
        loopMs = dt * 1000.0;
        flower.setUp();

        // ---- Buttons: read every edge exactly once per loop ----
        boolean aPressed = gamepad1.aWasPressed();
        boolean xPressed = gamepad1.xWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean rbPressed = gamepad1.rightBumperWasPressed();
        boolean up = gamepad1.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed();

        if (aPressed && poseReady) {
            aiming = !aiming;
            if (!aiming) turret.stop();
        }
        if (xPressed) sotm.setCompensationEnabled(!sotm.isCompensationEnabled());
        if (yPressed) {
            autoRail = !autoRail;
            railLockedOut = false;
        }
        if (bPressed) autoRail = false;
        if (!gamepad1.left_bumper && !autoRail) railLockedOut = false;

        if (up) SOTM.TOF_SCALE += 0.05;
        if (down) SOTM.TOF_SCALE = Math.max(0.1, SOTM.TOF_SCALE - 0.05);
        if (right) SOTM.TURRET_LEAD_SEC += 0.01;
        if (left) SOTM.TURRET_LEAD_SEC = Math.max(0.0, SOTM.TURRET_LEAD_SEC - 0.01);

        // ---- Sensors first: Turret kV and SOTM both use this Pinpoint update ----
        odo.update();
        boolean pinpointReady = poseReady
                && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;

        if (!pinpointReady) {
            drive.drive(0, 0, 0);
            autoRail = false;
            railWasActive = false;
            turret.stop();
            sotm.resetVelocity();
            if (launcher.getLaunchState() != Launcher.LaunchState.IDLE) launcher.cancelShot();
            shotArmed = false;
            intake.startIntake();
            launcher.update();
            setIndicator(false);
            telemetry.addData("Pinpoint", odo.getDeviceStatus());
            telemetry.addLine(poseReady ? "Pinpoint not READY" : "Pose not initialized: restart");
            telemetry.update();
            return;
        }

        // ---- Drive: rail mode or normal sticks ----
        Pose2D railPose = odo.getPosition();
        boolean railActive = (autoRail || gamepad1.left_bumper) && !railLockedOut;
        if (railActive) {
            if (!railWasActive && !startRail(railPose)) {
                railActive = false;
                autoRail = false;
                railLockedOut = true;
            } else if (!updateRail(dt, railPose, !autoRail)) {
                railActive = false;
                autoRail = false;
                railLockedOut = true;
            }
        }
        if (!railActive) {
            if (railLockedOut) {
                drive.drive(0, 0, 0);   // stay stopped after an abort until LB is released
            } else {
                if (railWasActive) railStatus = "off";
                drive.drive(DRIVE_SCALE * gamepad1.left_stick_x,
                        DRIVE_SCALE * gamepad1.left_stick_y,
                        DRIVE_SCALE * gamepad1.right_stick_x);
            }
        }
        railWasActive = railActive;

        // ---- SOTM aim + flywheel solution ----
        if (aiming) sotm.update();
        else { turret.stop(); sotm.updateNoAim(); }

        // Flywheel follows the VIRTUAL distance, slew limited so it can also change
        // during a burst without faking a shot dip.
        double model = sotm.getShooterVelocity();
        if (Double.isFinite(model)) {
            double maxStep = FLYWHEEL_SLEW_TICKS_PER_SEC2 * (dt > 0 && dt < 0.25 ? dt : 0.02);
            flywheelTarget += Math.max(-maxStep, Math.min(maxStep, model - flywheelTarget));
            launcher.setTargetVelocity(flywheelTarget);
        }

        // ---- Shooting (same launcher call as teleop.java) ----
        // RB arms a shot. The launcher is started ONCE, only when SOTM is ready; after that the
        // launcher runs its normal sequence (spin up -> open stopper -> feed) without being
        // interrupted. The gated launch(request, ready) is NOT used: it closes the stopper on
        // every one-loop flicker of "ready", so the stopper toggled without feeding a ball.
        boolean shotReady = aiming && sotm.isReadyToShoot();
        if (bPressed) {
            shotArmed = false;
            launcher.cancelShot();
        } else if (rbPressed) {
            shotArmed = true;
        }
        boolean fire = false;
        if (shotArmed && launcher.getLaunchState() == Launcher.LaunchState.IDLE
                && (shotReady || !REQUIRE_SOTM_READY)) {
            fire = true;
            shotArmed = false;
        }
        launcher.launch(fire);

        // Intake always on. During a shot the launcher drives the same motor itself
        // (0.8 power while feeding); in every other state keep it at full power.
        if (launcher.getLaunchState() == Launcher.LaunchState.IDLE) intake.startIntake();
        recordShots();
        setIndicator(shotReady && launcher.isAtTargetVelocity());

        // ---- Telemetry ----
        Pose2D pose = odo.getPosition();
        telemetry.addData("Rail", railStatus);
        telemetry.addData("RED_1  aim / comp", "%s / %s",
                aiming ? "ON" : "OFF", sotm.isCompensationEnabled() ? "SOTM" : "OFF");
        telemetry.addData("Shot", shotArmed ? "ARMED, waiting: " + sotm.getHoldReason()
                : sotm.getHoldReason());
        telemetry.addData("Launcher", "%s  shots %d", launcher.getLaunchState(), totalShots);
        telemetry.addData("Last shot", lastShot);
        telemetry.addLine();
        telemetry.addData("Robot X/Y/H", "%.1f / %.1f / %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Robot vx / vy in/s", "%.1f / %.1f",
                sotm.getVelocityX(), sotm.getVelocityY());
        telemetry.addData("Speed / accel", "%.1f in/s / %.0f in/s2",
                sotm.getSpeed(), sotm.getAcceleration());
        telemetry.addData("Radial / side in/s", "%.1f / %.1f (+ toward / + left)",
                sotm.getRadialSpeed(), sotm.getTangentialSpeed());
        telemetry.addData("Turn deg/s", "%.1f", sotm.getTurnRateDegPerSec());
        telemetry.addLine();
        telemetry.addData("Real / virtual dist in", "%.1f / %.1f",
                sotm.getDistanceInches(), sotm.getVirtualDistanceInches());
        telemetry.addData("Virtual goal X/Y", "%.1f / %.1f",
                sotm.getVirtualGoalX(), sotm.getVirtualGoalY());
        telemetry.addData("Flight time s", "%.3f (scale %.2f)",
                sotm.getTimeOfFlightSeconds(), SOTM.TOF_SCALE);
        telemetry.addData("Lead deg", "%.2f", sotm.getLeadDegrees());
        telemetry.addData("Bearing rate deg/s", "%.1f", sotm.getBearingRateDegPerSec());
        telemetry.addData("Turret lead s", "%.2f", SOTM.TURRET_LEAD_SEC);
        telemetry.addLine();
        telemetry.addData("Turret actual / aim / cmd", "%.1f / %.1f / %.1f",
                turret.getLastAngleDegrees(), sotm.getTargetTurretDegrees(),
                sotm.getCommandTurretDegrees());
        telemetry.addData("Aim error deg", "%.2f", sotm.getErrorDegrees());
        telemetry.addData("Flywheel model / cmd / actual", "%.0f / %.0f / %.0f",
                model, flywheelTarget, launcher.getVelocity());
        telemetry.addData("Flywheel at speed", launcher.isAtTargetVelocity());
        telemetry.addData("Loop ms", "%.1f", loopMs);
        telemetry.update();
    }

    // ======================================= Rail mode =======================================

    /** Initialize the rail setpoints at the robot's current pose. false = refused. */
    private boolean startRail(Pose2D pose) {
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double hErr = Math.toDegrees(wrap(Math.toRadians(RAIL_HEADING_DEG)
                - pose.getHeading(AngleUnit.RADIANS)));
        if (x < RAIL_X_MIN_IN || x > RAIL_X_MAX_IN) {
            railStatus = String.format("REFUSED: X %.1f outside %.0f..%.0f, drive inside first",
                    x, RAIL_X_MIN_IN, RAIL_X_MAX_IN);
            return false;
        }
        if (Math.abs(hErr) > RAIL_ABORT_HEADING_ERROR_DEG) {
            railStatus = String.format("REFUSED: heading off by %.0f deg, face 90 first", hErr);
            return false;
        }
        if (y > RAIL_Y_IN + 6.0) {
            railStatus = String.format("REFUSED: Y %.1f too far from rail Y %.0f", y, RAIL_Y_IN);
            return false;
        }
        railSetX = x;
        railSetY = y;
        railSetV = 0;
        railDir = (RAIL_X_MAX_IN - x) >= (x - RAIL_X_MIN_IN) ? 1 : -1;   // head to the far end
        return true;
    }

    /** One rail control step. false = aborted (motors already stopped). */
    private boolean updateRail(double dt, Pose2D pose, boolean manual) {
        double step = (dt > 0 && dt < 0.1) ? dt : 0.02;
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double h = pose.getHeading(AngleUnit.RADIANS);

        // 1. Desired speed along the row.
        double want;
        if (manual) {
            want = gamepad1.left_stick_x * RAIL_MANUAL_MAX_SPEED_IN_PER_SEC;
        } else {
            if (railSetX >= RAIL_X_MAX_IN - 0.5) railDir = -1;
            else if (railSetX <= RAIL_X_MIN_IN + 0.5) railDir = 1;
            want = railDir * RAIL_AUTO_SPEED_IN_PER_SEC;
        }
        // Brake so the setpoint stops at the X limits: v <= sqrt(2 a d).
        double ahead = want >= 0 ? RAIL_X_MAX_IN - railSetX : railSetX - RAIL_X_MIN_IN;
        double cap = Math.sqrt(2.0 * RAIL_ACCEL_IN_PER_SEC2 * Math.max(0.0, ahead));
        want = Math.signum(want) * Math.min(Math.abs(want), cap);
        // Acceleration limit (smooth for SOTM's accel gate too).
        double maxDv = RAIL_ACCEL_IN_PER_SEC2 * step;
        railSetV += Math.max(-maxDv, Math.min(maxDv, want - railSetV));
        railSetX = Math.max(RAIL_X_MIN_IN, Math.min(RAIL_X_MAX_IN, railSetX + railSetV * step));

        // 2. Ease Y from the start (7) to the rail row.
        double yErrToRail = RAIL_Y_IN - railSetY;
        double yStep = Math.min(Math.abs(yErrToRail), RAIL_Y_APPROACH_SPEED_IN_PER_SEC * step);
        double setVy = Math.abs(yErrToRail) > 1e-6 ? Math.signum(yErrToRail) * yStep / step : 0.0;
        railSetY += Math.signum(yErrToRail) * yStep;

        // 3. Errors and safety checks.
        double ex = railSetX - x;
        double ey = railSetY - y;
        double eh = wrap(Math.toRadians(RAIL_HEADING_DEG) - h);
        String abort = null;
        if (Math.abs(ex) > RAIL_ABORT_X_ERROR_IN) abort = String.format("X error %.1f in", ex);
        else if (Math.abs(ey) > RAIL_ABORT_Y_ERROR_IN) abort = String.format("Y error %.1f in", ey);
        else if (y < RAIL_MIN_Y_IN) abort = String.format("Y %.1f too close to wall", y);
        else if (Math.abs(Math.toDegrees(eh)) > RAIL_ABORT_HEADING_ERROR_DEG)
            abort = String.format("heading error %.0f deg", Math.toDegrees(eh));
        else if (x < RAIL_X_MIN_IN - 4.0 || x > RAIL_X_MAX_IN + 4.0)
            abort = String.format("X %.1f past limit", x);
        if (abort != null) {
            drive.drive(0, 0, 0);
            railStatus = "ABORT: " + abort + " (release LB / press Y to retry)";
            return false;
        }

        // 4. Field-frame power command -> robot frame (works at any heading).
        double fx = RAIL_KV * railSetV + RAIL_KP_X * ex;
        double fy = RAIL_KV * setVy + RAIL_KP_Y * ey;
        double forward = fx * Math.cos(h) + fy * Math.sin(h);
        double right = fx * Math.sin(h) - fy * Math.cos(h);
        double mag = Math.hypot(forward, right);
        // Static-friction boost: add RAIL_KS_DRIVE in the commanded direction whenever the
        // robot is supposed to be moving or is off position by more than the deadband.
        boolean mustMove = Math.abs(railSetV) > 0.5 || Math.abs(setVy) > 0.5
                || Math.hypot(ex, ey) > RAIL_POSITION_DEADBAND_IN;
        if (mustMove && mag > 1e-4) {
            double boosted = mag + RAIL_KS_DRIVE;
            forward *= boosted / mag;
            right *= boosted / mag;
            mag = boosted;
        }
        if (mag > RAIL_MAX_POWER) {
            forward *= RAIL_MAX_POWER / mag;
            right *= RAIL_MAX_POWER / mag;
        }
        double turnCcw = RAIL_KP_HEADING * eh;
        if (Math.abs(Math.toDegrees(eh)) > RAIL_HEADING_DEADBAND_DEG) {
            turnCcw += Math.signum(eh) * RAIL_KS_TURN;
        }
        turnCcw = Math.max(-RAIL_MAX_TURN_POWER, Math.min(RAIL_MAX_TURN_POWER, turnCcw));
        // Drivetrain.drive(x = strafe right, y = stick sign (-1 forward), rx = turn clockwise)
        drive.drive(right, -forward, -turnCcw);

        railStatus = String.format("%s  setX %.1f v %.1f  errX %.1f errY %.1f errH %.1f"
                        + "  pwr fwd %.2f right %.2f turn %.2f",
                manual ? "MANUAL" : "AUTO", railSetX, railSetV, ex, ey, Math.toDegrees(eh),
                forward, right, -turnCcw);
        return true;
    }

    private static double wrap(double r) { return Math.atan2(Math.sin(r), Math.cos(r)); }

    /** When the launcher counts a new ball, save the conditions it was fired under. */
    private void recordShots() {
        int count = launcher.getShotsFired();
        if (launcher.getLaunchState() == Launcher.LaunchState.LAUNCH && count > lastShotCount) {
            totalShots += count - lastShotCount;
            lastShot = String.format("#%d  rad %.0f side %.0f in/s  lead %.1f  err %.1f  fly %.0f",
                    totalShots, sotm.getRadialSpeed(), sotm.getTangentialSpeed(),
                    sotm.getLeadDegrees(), sotm.getErrorDegrees(), launcher.getVelocity());
        }
        lastShotCount = launcher.getLaunchState() == Launcher.LaunchState.LAUNCH ? count : 0;
    }

    private void setIndicator(boolean ready) {
        if (ready == greenOn) return;
        if (ready) rgb.setGreen();
        else rgb.setRed();
        greenOn = ready;
    }

    @Override public void stop() {
        if (rgb != null) rgb.setOff();
        if (drive != null) drive.drive(0, 0, 0);
        if (turret != null) turret.stop();
        if (launcher != null) { launcher.stopLauncher(); Hardware.feeder.setPower(0); }
        if (intake != null) intake.stopIntake();
    }
}
