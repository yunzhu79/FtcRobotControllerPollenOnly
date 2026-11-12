package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.TurretEncoder;

/**
 * BIOBUZZ: reads the standalone Melonbotics quadrature encoder on REV encoder port 0.
 * The encoder shares the hardware-map device "left_front_drive" with motor power 0.
 * No servos are commanded in this test.
 *
 * Gear conversion (owned by TurretEncoder):
 * 60T encoder gear drives 20T; coaxial 16T drives 96T turret.
 * turret revolutions / encoder revolution = (60/20)*(16/96) = 0.5.
 * For a 4096-tick encoder: 8192 ticks per turret revolution.
 * Do not reset the left-front motor encoder while tracking turret position.
 */
@TeleOp(name = "Turret Get Angle Test", group = "Test")
public class TurretGetAngleTest extends OpMode {
    private TurretEncoder encoder;

    @Override
    public void init() {
        Hardware.init(hardwareMap);
        encoder = new TurretEncoder();
        telemetry.addLine("Manually rotate the turret; no servo power is commanded.");
        telemetry.addLine("Quadrature angles are relative to the current encoder zero.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Same REV encoder port 0. Reading does not command left-front drive power.
        int rawTicks = Hardware.leftFrontDrive.getCurrentPosition();
        double turretDegrees = encoder.getPosition();

        telemetry.addData("Encoder ticks", rawTicks);
        telemetry.addData("Turret angle (deg)", "%.2f", turretDegrees);
        telemetry.addData("Turret angle (rad)", "%.4f", Math.toRadians(turretDegrees));
        telemetry.addData("Turret rotations", "%.4f", turretDegrees / 360.0);
        telemetry.addLine("Expected: 2048 ticks = 90 turret degrees (check direction).");
        telemetry.update();
    }
}
