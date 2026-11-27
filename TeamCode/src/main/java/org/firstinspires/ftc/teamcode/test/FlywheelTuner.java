package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Flywheel speed-controller tuner. Gamepad 1:
 *
 *   A            start / stop flywheel
 *   Dpad up/down target velocity +/- 25 ticks/s
 *   Dpad R/L     kV +/- step
 *   Y / B        kP +/- step
 *   Left bumper  toggle step size (coarse / fine)
 *   X (hold)     open stopper + run feeder (fire balls) to watch the speed dip and recovery
 *
 * Gains changed here are live for this run only. Copy the final kV / kP into Launcher.java.
 */
@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTuner extends OpMode {
    private Launcher launcher;
    private LaunchStopper stopper;

    private boolean running = false;
    private double target = 750;
    private boolean coarse = true;

    private boolean feeding = false;
    private double minVelocityDuringFeed = 0;
    private double lastDip = 0;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private double recoveryTime = -1;
    private boolean waitingForRecovery = false;

    @Override
    public void init() {
        Hardware.init(hardwareMap);
        launcher = new Launcher();
        stopper = new LaunchStopper();
        launcher.setTargetVelocity(target);
        launcher.stopLauncher();
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        double kVStep = coarse ? 0.00002 : 0.000002;
        double kPStep = coarse ? 0.0005 : 0.0001;

        if (gamepad1.leftBumperWasPressed()) coarse = !coarse;

        if (gamepad1.aWasPressed()) {
            running = !running;
            if (running) launcher.setTargetVelocity(target);
            else launcher.stopLauncher();
        }
        if (gamepad1.dpadUpWasPressed()) {
            target += 5;
            if (running) launcher.setTargetVelocity(target);
        }
        if (gamepad1.dpadDownWasPressed()) {
            target = Math.max(0, target - 5);
            if (running) launcher.setTargetVelocity(target);
        }
        if (gamepad1.dpadRightWasPressed()) Launcher.kV += kVStep;
        if (gamepad1.dpadLeftWasPressed())  Launcher.kV = Math.max(0, Launcher.kV - kVStep);
        if (gamepad1.yWasPressed()) Launcher.kP += kPStep;
        if (gamepad1.bWasPressed()) Launcher.kP = Math.max(0, Launcher.kP - kPStep);

        launcher.update();
        double v = launcher.getVelocity();

        // Hold X to fire: measure the speed dip and how long it takes to get back in tolerance.
        if (gamepad1.x && running) {
            if (!feeding) {
                feeding = true;
                minVelocityDuringFeed = v;
                feedTimer.reset();
                waitingForRecovery = false;
            }
            stopper.setOpen();
            Hardware.feeder.setPower(1.0);
            Hardware.intake.setPower(1.0);
            minVelocityDuringFeed = Math.min(minVelocityDuringFeed, v);
        } else {
            if (feeding) {
                feeding = false;
                lastDip = target - minVelocityDuringFeed;
                feedTimer.reset();
                waitingForRecovery = true;
            }
            stopper.setClose();
            Hardware.feeder.setPower(0.0);
            Hardware.intake.setPower(0.0);
        }
        if (waitingForRecovery && launcher.isAtTargetVelocity()) {
            recoveryTime = feedTimer.seconds();
            waitingForRecovery = false;
        }

        telemetry.addData("Running (A)", running);
        telemetry.addData("Target ticks/s", "%.0f", target);
        telemetry.addData("Velocity (control enc)", "%.0f", v);
        telemetry.addData("Error", "%.0f", target - v);
        telemetry.addData("launcher1 / launcher2", "%.0f / %.0f", launcher.getVelocity1(), launcher.getVelocity2());
        telemetry.addData("Power", "%.3f", launcher.getPower());
        telemetry.addData("Battery V", "%.2f", launcher.getBatteryVoltage());
        telemetry.addLine();
        telemetry.addData("kV (dpad R/L)", "%.6f", Launcher.kV);
        telemetry.addData("kP (Y/B)", "%.4f", Launcher.kP);
        telemetry.addData("Step (LB)", coarse ? "coarse" : "fine");
        telemetry.addLine();
        telemetry.addData("Last dip while feeding", "%.0f ticks/s", lastDip);
        telemetry.addData("Recovery after release", recoveryTime < 0 ? "-" : String.format("%.2f s", recoveryTime));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (launcher != null) launcher.stopLauncher();
        if (Hardware.feeder != null) Hardware.feeder.setPower(0);
        if (Hardware.intake != null) Hardware.intake.setPower(0);
    }
}
