package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumDriveFlywheelServo extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    FlywheelServo wheel = new FlywheelServo();
    double flywheelPower1 = 0;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private final double increment = 0.05;

    @Override
    public void init() {
        drive.init(hardwareMap);
        FlywheelServo.init(hardwareMap);
    }
    @Override
    public void start() {
        flywheelPower1 = 1;
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input2 = gamepad1.b;
//        boolean input3 = gamepad1.x;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        double flywheelPower = flywheelPower1;

        if (dpadUp && !dpadUpPrev) {
            flywheelPower1 += increment;
        }
        if (dpadDown && !dpadDownPrev) {
            flywheelPower1 -= increment;
        }
        dpadUpPrev = dpadUp;
        dpadDownPrev = dpadDown;
        telemetry.addData("Flywheel", flywheelPower1);


        drive.driveFieldRelative(y, x, turn);
        wheel.doubleA(input1, input2, flywheelPower);

    }
}
