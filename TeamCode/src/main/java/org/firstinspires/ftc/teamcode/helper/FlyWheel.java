package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FlyWheel {

    
    private DcMotorEx flyWheel;
    private OpMode opMode;

    public void init (OpMode opMode) {

        this.opMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void start(double power) {
        flyWheel.setPower(power);
    }
    public void stop() {
        flyWheel.setPower(0);
    }
    public void setPower(double power) {

        flyWheel.setPower(power);
    }

    public double getPower(){
        return flyWheel.getPower();

    }

    public double getVelocity(){
        return flyWheel.getVelocity();
    }

    }