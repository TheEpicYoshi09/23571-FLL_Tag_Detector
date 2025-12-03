package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    public DriveTrain(){
        leftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLeftMotor(double velocity) {
        leftMotor.setVelocity(velocity);
    }

    public void setRightMotor(double velocity) {
        rightMotor.setPower(velocity);
    }
}
