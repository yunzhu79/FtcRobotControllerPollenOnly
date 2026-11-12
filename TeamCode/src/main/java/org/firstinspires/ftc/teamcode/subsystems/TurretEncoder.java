package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TurretEncoder {
    // Quadrature counts for one complete revolution of the encoder shaft.
    private static final double ENCODER_TICKS_PER_REVOLUTION = 4096.0;

    // Gear train: servo/encoder shaft (60T) -> intermediate shaft (20T).
    // On the intermediate shaft, a 16T gear drives the 96T turret gear.
    private static final double ENCODER_GEAR_TEETH = 60.0;
    private static final double INTERMEDIATE_DRIVEN_GEAR_TEETH = 20.0;
    private static final double INTERMEDIATE_DRIVING_GEAR_TEETH = 16.0;
    private static final double TURRET_GEAR_TEETH = 96.0;

    // Turret revolutions for each encoder revolution:
    // (60 / 20) * (16 / 96) = 0.5
    private static final double TURRET_REVOLUTIONS_PER_ENCODER_REVOLUTION =
            (ENCODER_GEAR_TEETH / INTERMEDIATE_DRIVEN_GEAR_TEETH)
            * (INTERMEDIATE_DRIVING_GEAR_TEETH / TURRET_GEAR_TEETH);

    private final DcMotorEx turretEncoder;
    private double turretDegrees;

    public TurretEncoder() {
        // Encoder port 0 is associated with motor port 0 (left_front_drive).
        turretEncoder = Hardware.leftFrontDrive;
        turretDegrees = 0.0;
    }

    public void updatePosition() {
        int turretTicks = turretEncoder.getCurrentPosition();
        double encoderDegrees =
                (turretTicks / ENCODER_TICKS_PER_REVOLUTION) * 360.0;
        turretDegrees = encoderDegrees * TURRET_REVOLUTIONS_PER_ENCODER_REVOLUTION;
    }

    // Continuous turret angle in degrees, relative to the encoder counter's zero.
    public double getPosition() {
        updatePosition();
        return turretDegrees;
    }
}
