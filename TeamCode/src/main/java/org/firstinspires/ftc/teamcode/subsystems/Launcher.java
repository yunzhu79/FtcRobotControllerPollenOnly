package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        WAIT_FOR_STOPPER,
        LAUNCH
    }
    private final double FEED_TIME_SECONDS = 0.12;
    private final double STOPPER_OPEN_DELAY_SECONDS = 0.5;
    private final double STOP_SPEED = 0.0;
    private final double HALF_SPEED = 0.5;
    private final double FULL_SPEED = 1.0;
    private final double TARGET_VELOCITY = 780;
    private static LaunchState launchState;
    private ElapsedTime launchTimer = new ElapsedTime();
    private static DcMotorEx launcher1 = null;
    private static DcMotorEx launcher2 = null;
    private static LaunchStopper launchStopper = null;
    private static DcMotorEx feeder = null;
    private static DcMotorEx intake = null;

    public Launcher() {
        launcher1 = Hardware.launcher1;
        launcher2 = Hardware.launcher2;
        launchStopper = new LaunchStopper();
        feeder = Hardware.feeder;
        intake = Hardware.intake;
        launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 25));
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 25));
        launchState = LaunchState.IDLE;
    }

    public void startLauncher() {
        launcher1.setVelocity(TARGET_VELOCITY);
        launcher2.setVelocity(TARGET_VELOCITY);
    }

    public void stopLauncher() {
        launcher1.setVelocity(STOP_SPEED);
        launcher2.setVelocity(STOP_SPEED);
        launchStopper.setClose();
        launchState = LaunchState.IDLE;
    }

    public double getVelocity() {
        return (launcher1.getVelocity() + launcher2.getVelocity()) / 2;
    }

    public LaunchState getLaunchState() {
        return launchState;
    }

    public double getTargetVelocity() {
        return TARGET_VELOCITY;
    }

    public void launch(boolean shotRequested) {
        if (launchState == LaunchState.IDLE && shotRequested) {
            feeder.setPower(STOP_SPEED);
            intake.setPower(HALF_SPEED);
            launchStopper.setClose();
            launcher1.setVelocity(TARGET_VELOCITY);
            launcher2.setVelocity(TARGET_VELOCITY);
            launchState = LaunchState.SPIN_UP;
            launchTimer.reset();
        }

        if (launchState == LaunchState.SPIN_UP) {
            if (Math.abs(getVelocity() - TARGET_VELOCITY) < 20) {
                // Open the stopper first, but do not feed any balls yet.
                launchStopper.setOpen();
                launchState = LaunchState.WAIT_FOR_STOPPER;
                launchTimer.reset();
            }
        } else if (launchState == LaunchState.WAIT_FOR_STOPPER) {
            if (launchTimer.seconds() >= STOPPER_OPEN_DELAY_SECONDS) {
                // The stopper has had 0.5 seconds to move clear.
                feeder.setPower(FULL_SPEED);
                launchState = LaunchState.LAUNCH;
                launchTimer.reset();
            }
        } else if (launchState == LaunchState.LAUNCH) {
            if (launchTimer.seconds() >= 10 * FEED_TIME_SECONDS) {
                feeder.setPower(STOP_SPEED);
                launchStopper.setClose();
                intake.setPower(FULL_SPEED);
                launchState = LaunchState.IDLE;
                launchTimer.reset();
            }
        }
    }
}
