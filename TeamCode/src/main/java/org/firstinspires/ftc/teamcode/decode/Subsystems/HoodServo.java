package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodServo {

    private Servo hoodLeft;
    private Servo hoodRight;


    // private CRServo CRservo;

    public void init(HardwareMap hardwareMap) {

        // -------- Standard Servos --------
        hoodLeft  = hardwareMap.get(Servo.class, "hoodServoLeft");
        hoodRight = hardwareMap.get(Servo.class, "hoodServoRight");

        // Reverse one servo if mounted opposite
        hoodRight.setDirection(Servo.Direction.REVERSE);


        // CRservo = hardwareMap.get(CRServo.class, "hoodServo");
    }

    public void setHoodservo(double angle) {
        hoodLeft.setPosition(angle);
        hoodRight.setPosition(angle);
    }


    /*
    public void CRServo(double power, boolean isForward){
        if (isForward) { // goes down
            CRservo.setDirection(DcMotorSimple.Direction.FORWARD);
            CRservo.setPower(power);
        } else { // goes up
            CRservo.setDirection(DcMotorSimple.Direction.REVERSE);
            CRservo.setPower(power);
        }
    }

    public void CRStop(){
        CRservo.setPower(0);
    }

    public double getPower(){
        return CRservo.getPower();
    }
    */


    public double getPosition(){

        return hoodLeft.getPosition();
    }
}
