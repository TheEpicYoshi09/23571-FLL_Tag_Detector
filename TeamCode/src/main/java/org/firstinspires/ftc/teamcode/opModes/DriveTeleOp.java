package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name="Drive")
public class DriveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drivetrain = new DriveTrain();
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;

            double leftVelocity = y + x;
            double rightVelocity = y - x;

            drivetrain.setLeftMotor(leftVelocity);
            drivetrain.setRightMotor(rightVelocity);
        }
    }
}
