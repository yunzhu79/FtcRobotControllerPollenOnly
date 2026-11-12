package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing 3 configuration for the BIOBUZZ mecanum robot.
 *
 * Motor names and directions match Hardware.java.
 * Pinpoint offsets and Foresight gains require on-robot calibration.
 */
public class Constants {

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.backLeftName.set("left_back_drive");
        c.frontRightName.set("right_front_drive");
        c.backRightName.set("right_back_drive");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        // PROVISIONAL offsets: verify pod axes, signs and distances with AutoTune.
        // Previous Hardware.java used -1.5 instead of -2.5.
        c.xPodOffset.set(0.07);
        c.yPodOffset.set(-0.18);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.5257541040361273);
                Controller secondaryTranslationalForward = Controller.proportional(0.19425209303295599);
                Controller primaryTranslationalLateral = Controller.proportional(1.9711025259504287);
                Controller secondaryTranslationalLateral = Controller.proportional(0.7282697145091742);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014293142136844374));
                c.brake.set(Controller.proportionalFeedforward(0.012149170816317717));

                c.headingFeedback.set(Controller.proportional(9.667515276010326));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06353486000840024, 0.007291363901692231));

                c.linearBrakeCoefficients.set(Matrix.diag(0.01861501782187982, 0.036027891327233975));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.004240704885366515, 0.002614667174947206));

                c.maxAchievableForwardVelocity.set(65.36301657973976);
                c.maxAchievableStrafeVelocity.set(37.71429441569187);
                c.naturalForwardDeceleration.set(58.693948097653625);
                c.naturalStrafeDeceleration.set(73.09013832789427);
            }
    );

    /** Create the Pedro follower used by Auto.java. */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
