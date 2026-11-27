package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front_drive");
                c.backLeftName.set("left_back_drive");
                c.frontRightName.set("right_front_drive");
                c.backRightName.set("right_back_drive");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.41);
        c.yPodOffset.set(2.16);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4334167725161062);
                Controller secondaryTranslationalForward = Controller.proportional(0.16013591633524726);
                Controller primaryTranslationalLateral = Controller.proportional(0.5396401222662135);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19938260572773214);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012653287535747307));
                c.brake.set(Controller.proportionalFeedforward(0.01075529440538521));

                c.headingFeedback.set(Controller.proportional(6.699780275657012));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06388698760652062, 0.008695957750308372));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1287101827955722, 0.02236541719533611));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018737401850398804, 0.0032105450436104763));

                c.maxAchievableForwardVelocity.set(61.86227258697975);
                c.maxAchievableStrafeVelocity.set(47.4259254108711);
                c.naturalForwardDeceleration.set(48.61190909258807);
                c.naturalStrafeDeceleration.set(68.95293909915142);
            }
    );

}
