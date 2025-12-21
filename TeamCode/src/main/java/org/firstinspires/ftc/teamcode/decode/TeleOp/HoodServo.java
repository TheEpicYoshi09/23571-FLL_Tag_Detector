package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "hoodservo", group = "TeleOp")
public class HoodServo extends LinearOpMode {

    private Servo hoodservo;
    private double pos = 0.5;

    @Override
    public void runOpMode() {

        hoodservo = hardwareMap.get(Servo.class, "hoodservo");
        hoodservo.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                pos += 0.05;
            }

            if (gamepad1.dpad_down) {
                pos -= 0.05;
            }

            pos = Math.max(0, Math.min(1, pos));

            hoodservo.setPosition(pos);

            telemetry.addData("HoodServo Position", pos);
            telemetry.update();
        }
    }
}
