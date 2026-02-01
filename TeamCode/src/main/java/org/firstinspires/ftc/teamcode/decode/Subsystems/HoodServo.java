package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodServo {
    private Servo leftServo,rightServo;
    private CRServo CRservo;

    public void init(HardwareMap hardwareMap){
        //CRservo = hardwareMap.get(CRServo.class,"hoodServo");
       leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "leftServo");

        rightServo.setDirection(Servo.Direction.REVERSE);

    }
    public void setHoodservo(double angle){


       leftServo.setPosition(angle);
        rightServo.setPosition(angle);


    }

    public void CRServo(double power, boolean isForward){
//        if (isForward == true){// goes down
//            CRservo.setDirection(DcMotorSimple.Direction.FORWARD);
//            CRservo.setPower(power);
//        }
//        else if( isForward == false){ // goes up
//            CRservo.setDirection(DcMotorSimple.Direction.REVERSE);
//            CRservo.setPower(power);
//        }

    }

    public void CRStop(){
        //CRservo.setPower(0);
    }

    //public double getPower(){
//        double power = CRservo.getPower();
//        return power;
    //}
    public double getPosition(){
       double postion = leftServo.getPosition();
        return postion;
    }
}
