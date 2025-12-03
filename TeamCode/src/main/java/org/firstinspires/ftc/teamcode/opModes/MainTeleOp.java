package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="Drive")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drivetrain = new DriveTrain();
        Intake intake = new Intake();
        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;

            double leftVelocity = (y + x) * 20;
            double rightVelocity = (y - x) * 20;

            drivetrain.setLeftMotor(leftVelocity);
            drivetrain.setRightMotor(rightVelocity);


            if(gamepad1.xWasPressed()) intake.setMaxVelocity();
            if(gamepad1.aWasPressed()) intake.setZeroVelocity();
        }
    }
}
