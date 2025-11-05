package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flipper {
    private Servo flipper;
    private OpMode opMode;

    public static final double flipperStart = 1.0;
    public static final double flipperIncrement = -0.2222;
    //public static final double gateIntake = 0.6;
    //public static final String GATE_CLOSE = "GATE_CLOSE";
    //public static final String GATE_SHOOT = "GATE_SHOOT";
    //public static final String GATE_INTAKE = "GATE_INTAKE";
    //private String gatePosition;

    //private double flipperPos;

    public void init(HardwareMap hwMap) {
        flipper = hwMap.get(Servo.class, "flipper");
        //flipper.scaleRange(-flipperIncrement, flipperStart);
        this.opMode = opMode;
        resetFlipper();
    }
/*
    public void setGatePosition(String gatePosition){
        this.gatePosition = gatePosition;

        if(gatePosition.equals(GATE_CLOSE)){
            kickerPos.setPosition(gateClose);
        } else if(gatePosition.equals(GATE_SHOOT)){
            kickerPos.setPosition(gateShoot);
        }  else if(gatePosition.equals(GATE_INTAKE)){
            kickerPos.setPosition(gateIntake);
        } else {
            return;
        }

    }

    public String getGatePosition(){
        return gatePosition;
    }
*/
    public void setPosition(double position){
        flipper.setPosition(position);
        //flipperPos = position;
    }

    public void resetFlipper() {
        setPosition(flipperStart);
    }

    public void turnFlipper() {
        setPosition( Math.max(getPosition() + flipperIncrement, -flipperIncrement/2) );
    }
    public double getPosition(){
        return flipper.getPosition();
    }

}
