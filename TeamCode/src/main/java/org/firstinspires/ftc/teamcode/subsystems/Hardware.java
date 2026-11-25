package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hardware {
    public static GoBildaPinpointDriver odo = null;
    public static DcMotorEx leftFrontDrive = null;
    public static DcMotorEx rightFrontDrive = null;
    public static DcMotorEx leftBackDrive = null;
    public static DcMotorEx rightBackDrive = null;
    public static DcMotorEx launcher1 = null;
    public static DcMotorEx launcher2 = null;
    public static DcMotorEx intake = null;
    public static DcMotorEx feeder = null;
    public static Servo leftFlowerServo = null;
    public static Servo rightFlowerServo = null;
    public static Servo launchStopperServo = null;
    public static CRServo turretServo1 = null;
    public static CRServo turretServo2 = null;
    public static Servo rgbIndicator = null;

    public static void init(HardwareMap hardwareMap) {
        // AUTO bulk caching: hub reads (encoders, velocities) are cached and refreshed
        // automatically, so a loop does not pay one hub round-trip per read. Safe for every
        // OpMode (no clearBulkCache() calls needed). Read each encoder once per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        leftFlowerServo = hardwareMap.get(Servo.class, "left_flower_servo");
        rightFlowerServo = hardwareMap.get(Servo.class, "right_flower_servo");
        launchStopperServo = hardwareMap.get(Servo.class, "launch_stopper_servo");
        turretServo1 = hardwareMap.get(CRServo.class, "turret_servo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turret_servo2");
        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");

        leftFrontDrive.setDirection(REVERSE);
        rightFrontDrive.setDirection(FORWARD);
        leftBackDrive.setDirection(REVERSE);
        rightBackDrive.setDirection(FORWARD);
        launcher1.setDirection(FORWARD);
        launcher2.setDirection(REVERSE);
        intake.setDirection(REVERSE);
        feeder.setDirection(FORWARD);
        leftFlowerServo.setDirection(Servo.Direction.REVERSE);
        rightFlowerServo.setDirection(Servo.Direction.FORWARD);
        launchStopperServo.setDirection(Servo.Direction.REVERSE);
        turretServo1.setDirection(CRServo.Direction.FORWARD);
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher1.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        feeder.setZeroPowerBehavior(BRAKE);

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcher1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretServo1.setPower(0.0);
        turretServo2.setPower(0.0);

        odo.setOffsets(-1.5,0,DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }
}
