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
@Autonomous(name = "auto shoot and park", group = "Autonomous")
public class autoShootPark extends LinearOpMode {
    private enum Main { SHOOT1, TO_PARK, DONE }
    private enum Shot { SPINUP, WAIT_FOR_STOPPER, FEED, DONE }
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose FIRST_SHOOT_POS = P.of(55, 1, 90);
    private static final Pose PARK_POS = P.of(12, 108, 90);
    private static final double FEED_SECONDS = 1.2; // 10 * 0.12 in Launcher.java
    private static final double STOPPER_OPEN_DELAY_SECONDS = 0.5;

    private Follower follower;
    private InTake intake;
    private Launcher launcher;
    private LaunchStopper stopper;
    private FlowerIntake flower;
    private Turret turret;
    private StateMachine mainMachine, shotMachine;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private Path toPark;
    private int volleys;

    private static Path line(Pose a, Pose b) {
        return Paths.line(a, b).linear(a, b);
    }

    private void buildPaths() {
        toPark = line(FIRST_SHOOT_POS, PARK_POS).linear(FIRST_SHOOT_POS, PARK_POS);
    }

    private void drive(Path path) {
        pathTimer.reset();
        Hardware.feeder.setPower(0.0); // Never feed while driving.
        follower.follow(path);
    }

    private void startShot() {
        Hardware.feeder.setPower(0.0);
        shotMachine.reset();
        shotMachine.start();
    }

    private boolean shotFinished() {
        return shotMachine.getStateString().equals("DONE");
    }

    private void buildMachines() {
        // Independent, nonblocking shooting state machine; updated alongside the main machine.
        shotMachine = new StateMachineBuilder()
            .state(Shot.SPINUP)
            .onEnter(() -> {
                Hardware.feeder.setPower(0.0);
                Hardware.intake.setPower(0.8); // Match Launcher.launch() during spin-up
                stopper.setClose();
                shotTimer.reset();
                launcher.startLauncher(); // Target speed is owned by Launcher.java
            })
            .transition(launcher::isAtTargetVelocity, Shot.WAIT_FOR_STOPPER)
            .state(Shot.WAIT_FOR_STOPPER)
            .onEnter(() -> { stopper.setOpen(); shotTimer.reset(); })
            .transition(() -> shotTimer.seconds() >= STOPPER_OPEN_DELAY_SECONDS, Shot.FEED)
            .state(Shot.FEED)
            .onEnter(() -> { Hardware.feeder.setPower(1.0); shotTimer.reset(); })
            .transition(() -> shotTimer.seconds() >= FEED_SECONDS, Shot.DONE)
            .onExit(() -> {
                Hardware.feeder.setPower(0.0);
                stopper.setClose();
                volleys++;
            })
            .state(Shot.DONE)
            .build();

        mainMachine = new StateMachineBuilder()
            .state(Main.SHOOT1)
                .onEnter(this::startShot) // No initial driving.
                .transition(this::shotFinished, Main.TO_PARK)
            .state(Main.TO_PARK)
                .onEnter(() -> {
                    stopper.setClose(); // Keep pollen retained while flywheels spin.
                    flower.setUp();
                    drive(toPark);
                })
                .transition(() -> !follower.isBusy(), Main.DONE)
            .state(Main.DONE)
                .onEnter(this::stopMechanisms)
            .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap);
        intake = new InTake();
        launcher = new Launcher();
        stopper = new LaunchStopper();
        flower = new FlowerIntake();
        turret = new Turret();
        turret.setPIDF(2.0, 0.0, 0.0, 0.0);
        turret.setFeedforward(0.0, 0.0);
        turret.stop();
        intake.stopIntake();
        stopper.setClose();
        flower.setUp();
        follower = Constants.create(hardwareMap);
        follower.setPose(FIRST_SHOOT_POS);
        buildPaths();
        buildMachines();
        telemetry.update();
        waitForStart();
        if (isStopRequested()) { stopMechanisms(); return; }
        autoTimer.reset();
        launcher.startLauncher();
        // Hold the turret at its current startup angle, as in the tested TeleOp.
        turret.rotateByAngle(0.0);
        mainMachine.start();
        try {
            while (opModeIsActive()) {
                follower.update();
                Hardware.odo.update();
                turret.update();
                launcher.update(); // Flywheel speed control must run every loop.
                // Both machines progress each loop; drivetrain and launcher run concurrently.
                mainMachine.update();
                // Lower the first flower intake 1 second into the route, without blocking driving.
                if (shotMachine != null && !shotFinished()) shotMachine.update();
                telemetry.addData("Turret angle (deg)", "%.2f", turret.getAngleDegrees());
                telemetry.addData("Turret error (deg)", "%.2f", Math.toDegrees(turret.getError()));
                telemetry.addData("Main", mainMachine.getStateString());
                telemetry.addData("Garden route elapsed (s)", "%.1f", pathTimer.seconds());
                telemetry.addData("Shot", shotMachine.getStateString());
                telemetry.addData("Volleys attempted", volleys);
                telemetry.addData("Feeder power", "%.2f", Hardware.feeder.getPower());
                telemetry.addData("Time", "%.1f", autoTimer.seconds());
                telemetry.addData("Pose", "%.1f, %.1f", follower.pose().x(), follower.pose().y());
                telemetry.update();
                if (mainMachine.getStateString().equals("DONE")) break;
            }
        } finally { stopMechanisms(); }
    }

    private void stopMechanisms() {
        if (stopper != null) stopper.setClose();
        if (Hardware.feeder != null) Hardware.feeder.setPower(0.0);
        if (intake != null) intake.stopIntake();
        if (flower != null) flower.setUp();
        if (launcher != null) launcher.stopLauncher();
        if (turret != null) turret.stop();
        if (Hardware.leftFrontDrive != null) Hardware.leftFrontDrive.setPower(0);
        if (Hardware.leftBackDrive != null) Hardware.leftBackDrive.setPower(0);
        if (Hardware.rightFrontDrive != null) Hardware.rightFrontDrive.setPower(0);
        if (Hardware.rightBackDrive != null) Hardware.rightBackDrive.setPower(0);
    }
}
