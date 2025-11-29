package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {
    DcMotor leftMotor;
    DcMotor rightMotor;

    public DriveTrain(){
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLeftMotor(double power) {
        leftMotor.setPower(power);
    }

    public void setRightMotor(double power) {
        rightMotor.setPower(power);
    }
}
