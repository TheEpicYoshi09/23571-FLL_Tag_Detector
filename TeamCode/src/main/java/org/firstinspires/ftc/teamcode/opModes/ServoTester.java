package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="testOpMode")
public class ServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo testServo = hardwareMap.get(Servo.class, "testServo");

        waitForStart();

        while(opModeIsActive()){
            testServo.setPosition(0.0);
            sleep(500);
            testServo.setPosition(0.5);
            sleep(500);
            testServo.setPosition(0.25);
            sleep(500);
        }
    }
}
