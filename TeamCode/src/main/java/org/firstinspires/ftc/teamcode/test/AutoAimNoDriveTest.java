package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** Stationary-motor autoaim test: manually move robot while Pinpoint tracks pose. */
@TeleOp(name="AutoAim No Drive Test", group="Test")
public class AutoAimNoDriveTest extends OpMode {
    // Set to the robot's actual start pose in the SAME field coordinate system.
    public static double START_X_IN = 55.0;
    public static double START_Y_IN = 1.0;
    public static double START_HEADING_DEG = 90.0;
    private GoBildaPinpointDriver odo;
    private Turret turret;
    private FlowerIntake flowerIntake;
    private AutoAim aim;
    private boolean aiming = false;
    private boolean oldA;
    private boolean poseInitialized = false;
    private long lastLoopNanos = 0L;

    @Override public void init() {
        telemetry.setMsTransmissionInterval(50);
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

        turret = new Turret();
        // Controller gains come from the Turret defaults (kP/kD/kV/kS).
        flowerIntake = new FlowerIntake();
        flowerIntake.setUp(); // Keep flower intake stowed during INIT.
        turret.stop();
        aim = new AutoAim(AutoAim.Target.RED_1, turret);

        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Pose assigned", poseInitialized);
        telemetry.addData("Start pose requested", "%.1f / %.1f / %.1f",
                START_X_IN, START_Y_IN, START_HEADING_DEG);
        telemetry.addLine("START: aim RED_1 | Move robot by hand | A toggle aim | D-pad CELL");
        if (!poseInitialized) telemetry.addLine("Pinpoint not READY: STOP and re-INIT after checking device");
        telemetry.update();
    }

    @Override public void start() {
        flowerIntake.setUp();
        if (!poseInitialized) return;
        aiming = true;
    }

    @Override public void loop() {
        long nowNanos = System.nanoTime();
        double loopMs = lastLoopNanos == 0L ? 0.0 : (nowNanos - lastLoopNanos) * 1e-6;
        lastLoopNanos = nowNanos;
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
        if (!poseInitialized) {
            turret.stop();
            telemetry.addLine("Pinpoint pose not initialized. Restart OpMode.");
            telemetry.addData("Pinpoint status", odo.getDeviceStatus());
            telemetry.update();
            return;
        }
        if (aiming && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) aim.update();
        else turret.stop();

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
        telemetry.addData("OpMode loop ms", "%.1f", loopMs);
        telemetry.addData("Turret actual deg", "%.1f", turret.getLastAngleDegrees());
        telemetry.addData("Turret vel deg/s", "%.1f", Math.toDegrees(turret.getVelocity()));
        telemetry.addData("Turret target deg", "%.1f", aim.getTargetTurretDegrees());
        telemetry.addData("Turret error deg", "%.1f", aim.getErrorDegrees());
        telemetry.addData("Turret kP / kD / kV / kS", "%.3f / %.3f / %.3f / %.3f",
                turret.getKP(), turret.getKD(), turret.getKV(), turret.getKS());
        telemetry.addData("Within turret limits", aim.isWithinLimits());
        telemetry.addData("Aligned", aim.isAtTarget());
        telemetry.update();
    }

    @Override public void stop() {
        turret.stop();
    }
}
