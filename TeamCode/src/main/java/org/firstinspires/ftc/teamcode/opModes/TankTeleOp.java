package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TankTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drivetrain = new DriveTrain();
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;

            double leftPower = y + x;
            double rightPower = y - x;

            drivetrain.setLeftMotor(leftPower);
            drivetrain.setRightMotor(rightPower);
        }
    }
}
