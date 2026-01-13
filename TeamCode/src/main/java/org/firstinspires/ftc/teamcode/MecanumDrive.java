package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends OpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;


    @Override
    public void init() {
        // INITIALISE MOTORS
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeft");
        backLeftMotor   = hwMap.get(DcMotor.class, "backLeft");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRight");
        backRightMotor  = hwMap.get(DcMotor.class, "backRight");

        // REVERSING RIGHT SIDE MOTOR DIRECTION MAY NOT BE NECESSARY
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("DriveTrain init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        FWD = 

        // limit power if you want by dividing
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);

    }
}
