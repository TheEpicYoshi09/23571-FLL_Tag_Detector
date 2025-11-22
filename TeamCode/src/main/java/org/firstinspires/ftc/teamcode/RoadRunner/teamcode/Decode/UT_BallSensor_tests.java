package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_BallSensor_tests extends LinearOpMode {

    DC_BallSensor Balls = new DC_BallSensor(this);

    @Override
    public void runOpMode() throws InterruptedException {
        Balls.SensorInit();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("color", Balls.Sample());
            telemetry.addData("distance",Balls.Present());
            telemetry.update();
        }
    }
}
