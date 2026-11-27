package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.AutoDistance;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Auto distancing test (NO auto-aim): the driver aims the robot at the selected hive.
 * The turret is PIDF-locked at 0 deg (straight ahead) while the robot drives.
 * The flower intake is always UP.
 * Flywheel speed follows AutoDistance.getVelocity(robot pose, selected hive).
 * Hive coordinates come from AutoAim, so both OpModes always use the same values.
 *
 * Start pose: (55, 10.5) facing 90 deg, Pedro field coordinates.
 * The turret must be at its 0 deg position when INIT is pressed (encoder reference).
 *
 * Controls (gamepad1):
 *   Sticks         drive
 *   Right bumper   shoot (waits for flywheel at speed + robot still + turret locked)
 *   X              switch model: distance only <-> distance + angle
 *   D-pad          hive: UP Blue1 | RIGHT Blue2 | DOWN Red1 | LEFT Red2
 */
@TeleOp(name = "Auto Distance Test", group = "Test")
public class AutoDistanceTest extends OpMode {
    // Start pose (Pedro field coordinates).
    public static double START_X_IN = 55.0;
    public static double START_Y_IN = 10.5;
    public static double START_HEADING_DEG = 90.0;

    // Turret lock.
    public static double TURRET_LOCK_DEG = 0.0;
    public static double TURRET_TOLERANCE_DEG = 2.0;

    public static double MAX_SPEED_IN_S = 2.0;      // "still" for shooting
    public static double MAX_TURN_DEG_S = 5.0;

    private GoBildaPinpointDriver odo;
    private Drivetrain drive;
    private Turret turret;
    private Launcher launcher;
    private InTake intake;
    private FlowerIntake flowerIntake;

    private AutoAim.Target target = AutoAim.Target.RED_1;
    private boolean poseInitialized = false;
    private double commandedVelocity = Double.NaN;
    private boolean oldX;

    @Override public void init() {
        Hardware.init(hardwareMap);
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
            poseInitialized = true;
        }

        drive = new Drivetrain();
        turret = new Turret();
        // Same settings as AutoAimAutoShootTest: kV cancels robot rotation, kS beats friction.
        turret.setFeedforward(0.12, 0.10);
        turret.stop();
        launcher = new Launcher();
        intake = new InTake();
        flowerIntake = new FlowerIntake();
        flowerIntake.setUp();
        new LaunchStopper().setClose();

        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Start pose", "%.1f / %.1f / %.1f deg",
                START_X_IN, START_Y_IN, START_HEADING_DEG);
        telemetry.addData("Turret deg (should be ~0)", "%.1f", turret.getAngleDegrees());
        telemetry.addData("Model", AutoDistance.modelName());
        telemetry.addData("Hive", target);
        if (!poseInitialized) telemetry.addLine("Pinpoint not READY: STOP and re-INIT");
        telemetry.update();
    }

    @Override public void init_loop() {
        flowerIntake.setUp();
    }

    @Override public void start() {
        if (!poseInitialized) return;
        flowerIntake.setUp();
        intake.startIntake();
    }

    @Override public void loop() {
        if (flowerIntake == null) return;
        flowerIntake.setUp();   // always UP
        odo.update();           // before the turret: kV uses Pinpoint heading velocity

        if (!poseInitialized) {
            drive.drive(0, 0, 0);
            turret.stop();
            launcher.stopLauncher();
            telemetry.addLine("Pinpoint pose not initialized. Restart OpMode.");
            telemetry.update();
            return;
        }

        // ---------------- Buttons ----------------
        boolean x = gamepad1.x;
        if (x && !oldX) AutoDistance.USE_ANGLE_TERM = !AutoDistance.USE_ANGLE_TERM;
        oldX = x;
        if (gamepad1.dpad_up) target = AutoAim.Target.BLUE_1;
        else if (gamepad1.dpad_right) target = AutoAim.Target.BLUE_2;
        else if (gamepad1.dpad_down) target = AutoAim.Target.RED_1;
        else if (gamepad1.dpad_left) target = AutoAim.Target.RED_2;

        double tx, ty;
        switch (target) {
            case RED_2:  tx = AutoAim.RED_2_X_IN;  ty = AutoAim.RED_2_Y_IN;  break;
            case BLUE_1: tx = AutoAim.BLUE_1_X_IN; ty = AutoAim.BLUE_1_Y_IN; break;
            case BLUE_2: tx = AutoAim.BLUE_2_X_IN; ty = AutoAim.BLUE_2_Y_IN; break;
            case RED_1:
            default:     tx = AutoAim.RED_1_X_IN;  ty = AutoAim.RED_1_Y_IN;  break;
        }

        drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // ---------------- Turret lock (PIDF, every loop) ----------------
        turret.rotateUsingPIDF(Math.toRadians(TURRET_LOCK_DEG));
        double turretDeg = turret.getLastAngleDegrees();
        boolean turretLocked = Math.abs(turretDeg - TURRET_LOCK_DEG) <= TURRET_TOLERANCE_DEG;

        // ---------------- Auto distancing ----------------
        boolean ready = odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        Pose2D pose = odo.getPosition();
        double rx = pose.getX(DistanceUnit.INCH);
        double ry = pose.getY(DistanceUnit.INCH);
        double distance = AutoDistance.distance(rx, ry, tx, ty);
        double shotAngle = AutoDistance.angleDegrees(rx, ry, tx, ty);
        double modelVelocity = AutoDistance.getVelocity(rx, ry, tx, ty);
        boolean trusted = AutoDistance.isTrusted(rx, ry, tx, ty);

        // Only change speed while not feeding, so a target change mid-burst
        // doesn't look like a shot dip to the launcher.
        Launcher.LaunchState state = launcher.getLaunchState();
        boolean canRetarget = state == Launcher.LaunchState.IDLE
                || state == Launcher.LaunchState.SPIN_UP;
        if (ready && Double.isFinite(modelVelocity) && canRetarget) {
            commandedVelocity = modelVelocity;
            launcher.setTargetVelocity(commandedVelocity);   // also keeps the flywheel spinning
        }

        // ---------------- Shooting (driver aims the robot) ----------------
        double speed = Math.hypot(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH));
        double turnSpeed = Math.abs(Math.toDegrees(
                odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
        boolean stationary = speed <= MAX_SPEED_IN_S && turnSpeed <= MAX_TURN_DEG_S;

        launcher.launch(gamepad1.right_bumper, ready && stationary && turretLocked);

        // ---------------- Telemetry ----------------
        telemetry.addData("Hive (D-pad)", "%s (%.2f, %.2f)", target, tx, ty);
        telemetry.addData("Model (X to switch)", AutoDistance.modelName());
        telemetry.addData("Robot X / Y in", "%.1f / %.1f", rx, ry);
        telemetry.addData("Heading deg", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distance to hive in", "%.1f", distance);
        telemetry.addData("Shot angle deg (+ right)", "%.1f", shotAngle);
        telemetry.addData("Flywheel target", "%.0f", commandedVelocity);
        telemetry.addData("Flywheel actual", "%.0f", launcher.getVelocity());
        if (!trusted) telemetry.addLine("Outside measured range (40-75 in, +/-50 deg): speed is a guess");
        telemetry.addLine();
        telemetry.addData("Turret deg", "%.1f (lock %.1f)", turretDeg, TURRET_LOCK_DEG);
        telemetry.addData("Launcher state", state);
        if (!ready) telemetry.addLine("Pinpoint not READY");
        else if (!stationary) telemetry.addLine("Robot moving: shot waits");
        else if (!turretLocked) telemetry.addLine("Turret not at 0: shot waits");
        telemetry.addLine("RB shoot | X model | Dpad U Blue1 R Blue2 D Red1 L Red2");
        telemetry.update();
    }

    @Override public void stop() {
        if (drive != null) drive.drive(0, 0, 0);
        if (turret != null) turret.stop();
        if (launcher != null) { launcher.stopLauncher(); Hardware.feeder.setPower(0); }
        if (intake != null) intake.stopIntake();
    }
}
