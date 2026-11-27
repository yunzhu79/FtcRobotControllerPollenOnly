/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "teleop", group = "Linear OpMode")
public class teleop extends OpMode {
    private Drivetrain drivetrain = null;
    private InTake intake = null;
    private Launcher launcher = null;
    private FlowerIntake flowerIntake = null;
    private LaunchStopper launchStopper = null;
    private Turret turret = null;

    @Override
    public void init() {
        Hardware.init(hardwareMap);
        drivetrain   = new Drivetrain();
        intake       = new InTake();
        launcher     = new Launcher();
        flowerIntake = new FlowerIntake();
        launchStopper = new LaunchStopper();
        turret = new Turret();
        // Stronger position feedback for holding against disturbances.
        // Tune cautiously if oscillation occurs.
        turret.setPIDF(2.0, 0.0, 0.0, 0.0);
        turret.setFeedforward(0.0, 0.0);
        turret.stop();

        flowerIntake.setUp();
        launchStopper.setClose();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        intake.startIntake();
        launcher.startLauncher();
        turret.rotateByAngle(0.0);
    }

    @Override
    public void loop() {
        Hardware.odo.update();
        turret.update();

        // Normal driving - always active
        drivetrain.drive(gamepad1.left_stick_x,  gamepad1.left_stick_y,  gamepad1.right_stick_x);

        launcher.launch(gamepad1.rightBumperWasPressed());

        if (gamepad2.leftBumperWasPressed()) { launcher.startLauncher(); }
        if (gamepad2.rightBumperWasPressed()) { launcher.stopLauncher(); }
        if (gamepad2.yWasPressed()) { intake.startIntake(); }
        if (gamepad2.aWasPressed()) { intake.reverseIntake();}
        if (gamepad2.xWasPressed()) { intake.stopIntake(); }
        if (gamepad2.bWasPressed()) { flowerIntake.switchPosition(); }
        if (gamepad2.dpadDownWasPressed()) { flowerIntake.slightDown(); }
        if (gamepad2.dpadUpWasPressed()) { flowerIntake.slightUp(); }

        telemetry.addData("Turret encoder ticks", Hardware.leftFrontDrive.getCurrentPosition());
        telemetry.addData("Turret angle (deg)", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret error (deg)", "%.2f", Math.toDegrees(turret.getError()));
        telemetry.addData("Turret kP", "%.2f", turret.getKP());
        telemetry.addData("Launcher state", launcher.getLaunchState());
        telemetry.addData("Launcher target velocity", launcher.getTargetVelocity());
        telemetry.addData("Launcher velocity", launcher.getVelocity());
        telemetry.addData("Burst ended because", launcher.getLastEndReason());
        telemetry.addData("Burst length", "%.2f s", launcher.getBurstDuration());
        telemetry.addData("Feed pauses (last burst)", launcher.getPauseCount());
        telemetry.addData("Biggest dip (last burst)", "%.0f ticks/s", launcher.getMaxDip());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
