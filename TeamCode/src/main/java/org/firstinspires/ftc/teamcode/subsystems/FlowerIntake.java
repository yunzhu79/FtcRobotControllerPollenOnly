package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {
    public final double ZERO = 0.0;
    public final double DOWN = 0.02;
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

    public void setZero() {
        leftFlowerServo.setPosition(ZERO);
        rightFlowerServo.setPosition(ZERO);
        position = ZERO;
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

    public void slightUp() {
        if (position <= 1 && position >= 0) {
            position = position + 0.0005;
            leftFlowerServo.setPosition(position);
            rightFlowerServo.setPosition(position);
        }
    }

    public void slightDown() {
        if (position <= 1 && position >= 0) {
            position = position - 0.005;
            leftFlowerServo.setPosition(position);
            rightFlowerServo.setPosition(position);
        }
    }

    public void switchPosition() {
        if (Math.abs(position - DOWN) < Math.abs(position - UP) ) {
            leftFlowerServo.setPosition(UP);
            rightFlowerServo.setPosition(UP);
            position = UP;
        } else  {
            leftFlowerServo.setPosition(DOWN);
            rightFlowerServo.setPosition(DOWN);
            position = DOWN;
        }
    }

    public double getPosition() {
        return position;
    }
}
