package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * BIOBUZZ turret: initialize stationary, then press gamepad1 A ONCE
 * to request a +30 degree turret movement using encoder feedback and PIDF.
 * There are no other gamepad commands.
 */
@TeleOp(name = "Turret Rotate Test", group = "Test")
public class TurretRotateTest extends OpMode {
    private static final double TURN_ANGLE_DEGREES = -30.0;

    private Turret turret;
    private boolean moveRequested = false;
    private double targetDegrees = Double.NaN;

    @Override
    public void init() {
        Hardware.init(hardwareMap);

        turret = new Turret();
        turret.stop(); // Neutral power; PID is not running yet.

        moveRequested = false;
        targetDegrees = Double.NaN;
        showTelemetry();
    }

    @Override
    public void init_loop() {
        // Remain stationary during INIT.
        showTelemetry();
    }

    @Override
    public void loop() {
        // A can trigger only once during this OpMode run.
        if (!moveRequested && gamepad1.a) {
            double startingAngle = turret.getAngleDegrees();
            targetDegrees = startingAngle + TURN_ANGLE_DEGREES;
            turret.rotateByAngle(TURN_ANGLE_DEGREES);
            moveRequested = true;
        }

        if (moveRequested) {
            // PIDF must execute on every loop to reach and hold the target.
            turret.update();
        } else {
            // No PIDF output or movement commands before A.
            turret.stop();
        }

        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addData("A command sent", moveRequested);
        telemetry.addData("Requested delta (deg)", "%.2f", TURN_ANGLE_DEGREES);
        telemetry.addData("Target (deg)", Double.isNaN(targetDegrees)
                ? "not set" : String.format(java.util.Locale.US, "%.2f", targetDegrees));
        telemetry.addData("Encoder ticks", Hardware.leftFrontDrive.getCurrentPosition());
        telemetry.addData("Turret angle (deg)", "%.2f", turret.getAngleDegrees());
        telemetry.addData("PIDF", moveRequested ? "active" : "inactive");
        telemetry.addData("Static FF kS", "%.3f", turret.getKS());
        if (moveRequested) {
            telemetry.addData("Position error (deg)", "%.2f", Math.toDegrees(turret.getError()));
            telemetry.addData("At target (+/- 2 deg)", turret.atTarget(Math.toRadians(2.0)));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
    }
}
