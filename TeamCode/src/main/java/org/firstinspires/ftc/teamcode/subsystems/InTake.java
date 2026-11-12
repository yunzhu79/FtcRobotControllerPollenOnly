package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class InTake {
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private static DcMotorEx intake = null;

    public InTake() {
        intake = Hardware.intake;
    }

    public void startIntake() {
        intake.setPower(FULL_SPEED);
    }

    public void stopIntake() {
        intake.setPower(STOP_SPEED);
    }

    public void reverseIntake() {
        intake.setPower(-FULL_SPEED);
    }
}
