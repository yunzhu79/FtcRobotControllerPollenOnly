package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Drivetrain {
    private static DcMotorEx leftFrontDrive;
    private static DcMotorEx leftBackDrive;
    private static DcMotorEx rightFrontDrive;
    private static DcMotorEx rightBackDrive;

    //Initialization Method:
    public Drivetrain() {
        leftFrontDrive = Hardware.leftFrontDrive;
        leftBackDrive = Hardware.leftBackDrive;
        rightFrontDrive = Hardware.rightFrontDrive;
        rightBackDrive = Hardware.rightBackDrive;

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void drive(double x1, double y1, double rx1, double x2, double y2, double rx2) {
        double x = x1 + x2;
        double y = y1 + y2;
        double rx = rx1 + rx2;

        y = -y;
        rx = 0.8 * rx;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }
}