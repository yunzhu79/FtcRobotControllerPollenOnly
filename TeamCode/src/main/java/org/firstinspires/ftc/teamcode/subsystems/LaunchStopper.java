package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class LaunchStopper {
    public final double CLOSE = 0;
    public final double OPEN = 0.3;
    private static Servo launchStopperServo = null;
    private static double position;

    public LaunchStopper() {
        launchStopperServo = Hardware.launchStopperServo;

        launchStopperServo.setPosition(CLOSE);
        position = CLOSE;
    }

    public void setClose() {
        launchStopperServo.setPosition(CLOSE);
        position = CLOSE;
    }

    public void setOpen() {
        launchStopperServo.setPosition(OPEN);
        position = OPEN;
    }

    public double getPosition() {
        return position;
    }
}
