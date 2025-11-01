package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PushBar;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    PushBar bar = new PushBar();
    IntakeControl intake = new IntakeControl();
    List<Double> barPositions = new ArrayList<>(2);
    @Override
    public  void init() {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void start(){
        bar.pushBall(0.7, 1.0);
        barPositions = bar.getBarPosition();
        telemetry.addData("Left Bar, Right Bar: ", barPositions);
        telemetry.update();
    }

    @Override
    public void loop() {
        // use gamepad sticks to control driving
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);

        // press button to swing bars, release to return
        if(gamepad1.right_bumper) {
            bar.pushBall(0.5, 0.1);
            telemetry.addData("after push", bar.getBarPosition());
            telemetry.update();
        }
        else {
            bar.release(0.7, 1.0);
            telemetry.addData("after release", bar.getBarPosition());
            telemetry.update();
        }

        // activate intake
        if(gamepad1.left_bumper) {
            intake.setIntakePower(0.6);
        }
        else {
            intake.setIntakePower(0.0);
        }
    }

    @Override
    public void stop() {
        bar.pushBall(0.7, 1.0);
        telemetry.addData("Stop", bar.getBarPosition());
        telemetry.update();
    }
}
