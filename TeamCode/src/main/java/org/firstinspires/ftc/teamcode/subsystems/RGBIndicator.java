package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class RGBIndicator {
    private final double GREEN = 0.5;
    private final double BLUE = 0.61;
    private final double RED = 0.28;
    private final double WHITE = 0.9;
    private final double OFF = 0;
    private static Servo rgbIndicator = null;
    private static double color;

    public RGBIndicator() {
        rgbIndicator = Hardware.rgbIndicator;
        setOff();
    }

    public void setOff() {
        rgbIndicator.setPosition(OFF);
        color = OFF;
    }

    public void setBlue() {
        rgbIndicator.setPosition(BLUE);
        color = BLUE;
    }

    public void setGreen() {
        rgbIndicator.setPosition(GREEN);
        color = GREEN;
    }

    public void setRed() {
        rgbIndicator.setPosition(RED);
        color = RED;
    }

    public void setWhite() {
        rgbIndicator.setPosition(WHITE);
        color = WHITE;
    }

    public double getColor() {
        return color;
    }
}
