package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Kicker {

    private Servo kickerPos;
    private OpMode opMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;
        kickerPos = opMode.hardwareMap.get(Servo.class, "kicker");


    }

    public void setKickerPos(double position){
        kickerPos.setPosition(position);
    }
}
