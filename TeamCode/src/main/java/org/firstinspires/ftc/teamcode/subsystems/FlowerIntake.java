package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {
    public final double DOWN = 0;
    public final double UP = 0.45;
    private static Servo leftFlowerServo = null;
    private static Servo rightFlowerServo = null;
    private static double position;

    public FlowerIntake() {
        leftFlowerServo = Hardware.leftFlowerServo;
        rightFlowerServo = Hardware.rightFlowerServo;

        leftFlowerServo.setPosition(DOWN);
        rightFlowerServo.setPosition(DOWN);
        position = DOWN;
    }

    public void setDown() {
        leftFlowerServo.setPosition(DOWN);
        rightFlowerServo.setPosition(DOWN);
        position = DOWN;
    }

    public void setUp() {
        leftFlowerServo.setPosition(UP);
        rightFlowerServo.setPosition(UP);
        position = UP;
    }

    public void switchPosition() {
        if (position == DOWN) {
            leftFlowerServo.setPosition(UP);
            rightFlowerServo.setPosition(UP);
            position = UP;
        } else if (position == UP) {
            leftFlowerServo.setPosition(DOWN);
            rightFlowerServo.setPosition(DOWN);
            position = DOWN;
        }
    }

    public double getPosition() {
        return position;
    }
}
