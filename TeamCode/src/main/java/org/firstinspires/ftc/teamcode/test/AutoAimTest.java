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
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** Pinpoint-only autoaim and launcher shooting test. */
@TeleOp(name="AutoAim Test", group="Test")
public class AutoAimTest extends OpMode {
    // Set to the robot's actual start pose in the SAME field coordinate system.
    public static double START_X_IN = 55.0;
    public static double START_Y_IN = 1.0;
    public static double START_HEADING_DEG = 90.0;
    public static double FIXED_LAUNCHER_TICKS_PER_SEC = 750.0;
    private GoBildaPinpointDriver odo;
    private Drivetrain drive;
    private Turret turret;
    private Launcher launcher;
    private InTake intake;
    private FlowerIntake flowerIntake;
    private AutoAim aim;
    private boolean aiming = false;
    private boolean oldA;
    private boolean poseInitialized = false;

    @Override public void init() {
        Hardware.init(hardwareMap);
        odo = Hardware.odo;
        // Offsets, encoder resolution and directions are configured in Hardware.init().
        // Reset the Pinpoint exactly once, while the robot is stationary.
        odo.resetPosAndIMU();

        // Allow the Pinpoint IMU to finish its reset/calibration before
        // assigning the nonzero field pose. Do not reset again afterward.
        try {
            Thread.sleep(1200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            telemetry.addLine("Pinpoint initialization interrupted");
            telemetry.update();
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
        launcher = new Launcher();
        intake = new InTake();
        flowerIntake = new FlowerIntake();
        flowerIntake.setUp(); // Keep flower intake stowed during INIT.
        new LaunchStopper().setClose();
        turret.stop();
        aim = new AutoAim(AutoAim.Target.RED_1, turret);

        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Pose assigned", poseInitialized);
        telemetry.addData("Start pose requested", "%.1f / %.1f / %.1f",
                START_X_IN, START_Y_IN, START_HEADING_DEG);
        telemetry.addLine("START: intake + launcher + aim | A toggle aim | D-pad CELL | RB shoot");
        if (!poseInitialized) telemetry.addLine("Pinpoint not READY: STOP and re-INIT after checking device");
        telemetry.update();
    }

    @Override public void start() {
        flowerIntake.setUp();
        if (!poseInitialized) return;
        intake.startIntake();
        launcher.setTargetVelocity(FIXED_LAUNCHER_TICKS_PER_SEC);
        launcher.startLauncher();
        aiming = true;
    }

    @Override public void loop() {
        flowerIntake.setUp(); // Reassert UP every loop; no gamepad control.
        boolean a = gamepad1.a;
        if (a && !oldA && poseInitialized) { aiming = !aiming; if (!aiming) turret.stop(); }
        // D-pad selection is mutually exclusive and updates every loop.
        if (gamepad1.dpad_up) aim.setTarget(AutoAim.Target.BLUE_1);
        else if (gamepad1.dpad_right) aim.setTarget(AutoAim.Target.BLUE_2);
        else if (gamepad1.dpad_down) aim.setTarget(AutoAim.Target.RED_1);
        else if (gamepad1.dpad_left) aim.setTarget(AutoAim.Target.RED_2);
        oldA = a;
        odo.update();
        if (poseInitialized) drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (!poseInitialized) {
            drive.drive(0, 0, 0);
            turret.stop();
            telemetry.addLine("Pinpoint pose not initialized. Restart OpMode.");
            telemetry.addData("Pinpoint status", odo.getDeviceStatus());
            telemetry.update();
            return;
        }
        if (aiming && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) aim.update();
        else turret.stop();

        // Match teleop.java: update launcher shooting state machine every loop.
        // A right-bumper press triggers a shot; holding it does not retrigger.
        launcher.launch(gamepad1.rightBumperWasPressed());
        Pose2D pose = odo.getPosition();
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Selected CELL", aim.getTarget());
        telemetry.addLine("Dpad UP Blue1 | RIGHT Blue2 | DOWN Red1 | LEFT Red2");
        telemetry.addData("Aim enabled", aiming);
        telemetry.addLine("Flower intake: commanded UP");
        telemetry.addData("Target configured", aim.hasValidTarget());
        telemetry.addData("Robot X/Y in", "%.1f / %.1f", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        telemetry.addData("Robot heading deg", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Hive distance in", "%.1f", aim.getDistanceInches());
        telemetry.addData("Hive bearing deg", "%.1f", aim.getBearingDegrees());
        telemetry.addData("Turret actual deg", "%.1f", turret.getAngleDegrees());
        telemetry.addData("Turret target deg", "%.1f", aim.getTargetTurretDegrees());
        telemetry.addData("Turret error deg", "%.1f", aim.getErrorDegrees());
        telemetry.addData("Turret kP / kV / kS", "%.2f / %.2f / %.2f",
                turret.getKP(), turret.getKV(), turret.getKS());
        telemetry.addData("Within turret limits", aim.isWithinLimits());
        telemetry.addData("Aligned", aim.isAtTarget());
        telemetry.addData("Launcher state", launcher.getLaunchState());
        telemetry.addLine("Right bumper: shoot");
        telemetry.addData("Flywheel running", poseInitialized);
        telemetry.addData("Flywheel target ticks/s", "%.0f", FIXED_LAUNCHER_TICKS_PER_SEC);
        telemetry.addData("Flywheel actual ticks/s", "%.0f", launcher.getVelocity());
        telemetry.update();
    }

    @Override public void stop() {
        drive.drive(0, 0, 0);
        turret.stop();
        launcher.stopLauncher();
        intake.stopIntake();
    }
}
