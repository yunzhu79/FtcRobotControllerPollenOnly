package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.SOTM;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** SOTM driving and live shooting test. */
@TeleOp(name="SOTM Test", group="Test")
public class SOTMTest extends OpMode {
    public static double START_X_IN=55, START_Y_IN=1, START_HEADING_DEG=90;
    public static double FIXED_LAUNCHER_TICKS_PER_SEC=750;
    public static boolean ENABLE_SHOOTING=true; // Live shooting with right bumper
    private GoBildaPinpointDriver odo;
    private Drivetrain drive;
    private Turret turret;
    private Launcher launcher;
    private InTake intake;
    private FlowerIntake flower;
    private SOTM sotm;
    private boolean poseReady=false, aiming=false, oldA=false;

    @Override public void init() {
        Hardware.init(hardwareMap);
        odo=Hardware.odo;
        odo.resetPosAndIMU();
        try { Thread.sleep(1200); }
        catch (InterruptedException ex) { Thread.currentThread().interrupt(); return; }
        odo.update();
        if (odo.getDeviceStatus()==GoBildaPinpointDriver.DeviceStatus.READY) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH,START_X_IN,START_Y_IN,
                    AngleUnit.DEGREES,START_HEADING_DEG));
            odo.update(); poseReady=true;
        }
        drive=new Drivetrain();
        turret=new Turret();
        launcher=new Launcher();
        intake=new InTake();
        flower=new FlowerIntake();
        flower.setUp();
        turret.stop();
        sotm=new SOTM(AutoAim.Target.RED_1,turret);
        telemetry.addData("Pinpoint READY",poseReady);
        telemetry.addLine("A toggle aim | Dpad select CELL | RB shoot | B cancel");
        telemetry.addLine("Start with wheels lifted; verify turret sign and limits");
        telemetry.update();
    }
    @Override public void start() {
        if (flower == null || !poseReady) return;
        flower.setUp();
        aiming=true;
        sotm.resetVelocity();
        if (ENABLE_SHOOTING) {
            intake.startIntake();
            launcher.setTargetVelocity(FIXED_LAUNCHER_TICKS_PER_SEC);
            launcher.startLauncher();
        }
    }
    @Override public void loop() {
        if (flower == null) return;
        flower.setUp();
        boolean a=gamepad1.a;
        if (a && !oldA && poseReady) {
            aiming=!aiming;
            if (!aiming) turret.stop();
        }
        oldA=a;
        if (launcher.getLaunchState() == Launcher.LaunchState.IDLE) {
        if (gamepad1.dpad_up) sotm.setTarget(AutoAim.Target.BLUE_1);
        else if (gamepad1.dpad_right) sotm.setTarget(AutoAim.Target.BLUE_2);
        else if (gamepad1.dpad_down) sotm.setTarget(AutoAim.Target.RED_1);
        else if (gamepad1.dpad_left) sotm.setTarget(AutoAim.Target.RED_2);
        }
        odo.update();
        boolean ready=poseReady && odo.getDeviceStatus()==GoBildaPinpointDriver.DeviceStatus.READY;
        if (ready) drive.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        else drive.drive(0,0,0);
        if (ready && aiming) sotm.update(launcher.getFeederElapsedSeconds());
        else { turret.stop(); sotm.resetVelocity(); }
        // Match the working teleop.java: one edge-triggered request,
        // and advance the launcher state machine every loop.
        if (ENABLE_SHOOTING && ready) {
            if (gamepad1.bWasPressed()) {
                launcher.cancelShot();
                intake.startIntake();
            } else {
                launcher.launch(gamepad1.rightBumperWasPressed());
            }
        } else if (launcher.getLaunchState() != Launcher.LaunchState.IDLE) {
            launcher.cancelShot();
        }
        telemetry.addData("Launcher state", launcher.getLaunchState());
        telemetry.addData("Flywheel ready", launcher.isAtTargetVelocity());
        telemetry.addData("Flywheel ticks/s", launcher.getVelocity());
        telemetry.addData("Feeder elapsed (s)", launcher.getFeederElapsedSeconds());
        telemetry.addData("Prediction delay (s)", sotm.getPredictionDelaySeconds());
        telemetry.addData("Pinpoint",odo.getDeviceStatus());
        telemetry.addData("CELL",sotm.getTarget());
        telemetry.addData("Aim enabled",aiming);
        telemetry.addData("Velocity ready",sotm.isVelocityReady());
        telemetry.addData("Robot vx/vy (in/s)","%.1f / %.1f",sotm.getVelocityX(),sotm.getVelocityY());
        telemetry.addData("Robot speed (in/s)","%.1f",sotm.getSpeed());
        telemetry.addData("Distance (in)","%.1f",sotm.getDistanceInches());
        telemetry.addData("Estimated flight time (s)","%.3f",sotm.getFlightTimeSeconds());
        telemetry.addData("Lead angle (deg)","%.2f",sotm.getLeadDegrees());
        telemetry.addData("Turret actual / target (deg)","%.1f / %.1f",
                turret.getAngleDegrees(),sotm.getTargetTurretDegrees());
        telemetry.addData("Turret error (deg)","%.2f",sotm.getErrorDegrees());
        telemetry.addData("Within limits",sotm.isWithinLimits());
        telemetry.addData("Aligned",sotm.isAtTarget());
        telemetry.addData("Shooting enabled",ENABLE_SHOOTING);
        telemetry.addData("Flower","UP");
        telemetry.update();
    }
    @Override public void stop() {
        if (drive!=null) drive.drive(0,0,0);
        if (turret!=null) turret.stop();
        if (launcher!=null) launcher.stopLauncher();
        if (intake!=null) intake.stopIntake();
    }
}
