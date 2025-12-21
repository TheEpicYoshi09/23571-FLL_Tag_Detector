package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShooterAngleTest", group = "Testing")
public class ShooterAngleTest extends LinearOpMode {

    private ShooterAngle shooterAngle;

    @Override
    public void runOpMode() {

        shooterAngle = new ShooterAngle();
        shooterAngle.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double pos = shooterAngle.getPosition();

            if (gamepad1.dpad_up) {
                pos += 0.05;
            }

            if (gamepad1.dpad_down) {
                pos -= 0.05;
            }

            // keep between 0–1
            pos = Math.max(0, Math.min(1, pos));

            shooterAngle.setAngle(pos);

            telemetry.addData("Servo Pos", pos);
            telemetry.update();
        }
    }
}
