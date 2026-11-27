package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.InTake;
import org.firstinspires.ftc.teamcode.subsystems.LaunchStopper;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** BIOBUZZ: one alliance-independent local path frame; estimated robot-center poses. */
@Autonomous(name = "auto just park", group = "Autonomous")
public class autoJustPark extends LinearOpMode {
    private enum Main { TO_PARK, DONE }
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START_POS = P.of(78, 7, 90);
    private static final Pose PARK_POS = P.of(120, 30, 0);

    private Follower follower;
    private FlowerIntake flower;
    private Turret turret;
    private StateMachine mainMachine;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final double TO_PARK_TIMEOUT = 3.0;
    private Path toPark;

    private static Path line(Pose a, Pose b) {
        return Paths.line(a, b).linear(a, b);
    }

    private void buildPaths() {
        toPark = line(START_POS, PARK_POS).linear(START_POS, PARK_POS);
    }

    private void drive(Path path) {
        pathTimer.reset();
        Hardware.feeder.setPower(0.0); // Never feed while driving.
        follower.follow(path);
    }

    private void buildMachines() {
        mainMachine = new StateMachineBuilder()
            .state(Main.TO_PARK)
                .onEnter(() -> {
                    drive(toPark);
                    flower.setUp();
                })
                .transition(() -> !follower.isBusy() || pathTimer.seconds() >= TO_PARK_TIMEOUT, Main.DONE)
            .state(Main.DONE)
                .onEnter(this::stopMechanisms)
            .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap);
        flower = new FlowerIntake();
        turret = new Turret();
        turret.setPIDF(2.0, 0.0, 0.0, 0.0);
        turret.setFeedforward(0.0, 0.0);
        flower.setUp();
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POS);
        buildPaths();
        buildMachines();
        telemetry.update();
        waitForStart();
        if (isStopRequested()) { stopMechanisms(); return; }
        autoTimer.reset();
        turret.rotateByAngle(0.0);
        mainMachine.start();
        try {
            while (opModeIsActive()) {
                follower.update();
                Hardware.odo.update();
                // Both machines progress each loop; drivetrain and launcher run concurrently.
                mainMachine.update();
                // Lower the first flower intake 1 second into the route, without blocking driving.
                telemetry.addData("Main", mainMachine.getStateString());
                telemetry.addData("Time", "%.1f", autoTimer.seconds());
                telemetry.addData("Pose", "%.1f, %.1f", follower.pose().x(), follower.pose().y());
                telemetry.update();
                if (mainMachine.getStateString().equals("DONE")) break;
            }
        } finally { stopMechanisms(); }
    }

    private void stopMechanisms() {
        if (Hardware.feeder != null) Hardware.feeder.setPower(0.0);
        if (Hardware.leftFrontDrive != null) Hardware.leftFrontDrive.setPower(0);
        if (Hardware.leftBackDrive != null) Hardware.leftBackDrive.setPower(0);
        if (Hardware.rightFrontDrive != null) Hardware.rightFrontDrive.setPower(0);
        if (Hardware.rightBackDrive != null) Hardware.rightBackDrive.setPower(0);
    }
}
