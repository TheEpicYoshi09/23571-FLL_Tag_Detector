
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "doubleShootingMechanismStrong", group = "Test")
//    @Config // Enables configuration via FTC Dashboard
public class doubleShootingMechanismStrong68 extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;


    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor1.setPower(0.8);
            }
            else if (gamepad1.b) {
                motor1.setPower(-0.8);
            }
            else {
                motor1.setPower(0.0);
            }

            if (gamepad1.x) {
                motor2.setPower(0.8);
            }
            else if (gamepad1.y) {
                motor2.setPower(-0.8);
            }
            else {
                motor2.setPower(0.0);
            }


            if (gamepad1.dpad_down) {
                motor1.setPower(0.6);
            }
            else if (gamepad1.dpad_left) {
                motor1.setPower(-0.6);
            }
            else {
                motor1.setPower(0.0);
            }

            if (gamepad1.dpad_right) {
                motor2.setPower(0.6);
            }
            else if (gamepad1.dpad_up) {
                motor2.setPower(-0.6);
            }
            else {
                motor2.setPower(0.0);
            }
        }

    }
}
