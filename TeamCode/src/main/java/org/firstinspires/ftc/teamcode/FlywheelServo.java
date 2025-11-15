package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class FlywheelServo {
    private static DcMotor FlywheelMotor;
    private static CRServo leftServo;
    private static CRServo rightServo;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean buttonPressed = false;
    private boolean movementStarted = false;
    private boolean movementCompleted = false;
    boolean motorRunning = false;
    boolean lastButtonState = false;

    public static void init(HardwareMap hwMap) {
        FlywheelMotor = hwMap.get(DcMotor.class, "flywheel");
        leftServo = hwMap.get(CRServo.class, "leftServo");
        rightServo = hwMap.get(CRServo.class, "rightServo");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightServo.setDirection(CRServo.Direction.REVERSE);

    }
    public void spin(boolean input1, boolean input2, boolean input3) {
        if (input1) {
            FlywheelMotor.setPower(0.65);
        }
        if (input3) {
            FlywheelMotor.setPower(0);
        }
        if (input2 && !buttonPressed) {
            buttonPressed = true;
            timer.reset();
            movementStarted = true;
            movementCompleted = false;
        }

        if (movementStarted && !movementCompleted) {
            if (timer.seconds() < 0.25) { //change value in (sec)
                leftServo.setPower(1.0);
                rightServo.setPower(1.0);
            } else {
                leftServo.setPower(0.0);
                rightServo.setPower(0.0);
                movementCompleted = true;
                movementStarted = false;
                buttonPressed = false;
            }
        }
    }
    public void doubleA(boolean input1, boolean input2, double flywheelPower) {

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        FlywheelMotor.setPower(motorRunning ? flywheelPower : 0.0);
        lastButtonState = input1;

        if (input2 && !buttonPressed) {
            buttonPressed = true;
            timer.reset();
            movementStarted = true;
            movementCompleted = false;
        }

        if (movementStarted && !movementCompleted) {
            if (timer.seconds() < 0.10) { //change value in (sec)
                leftServo.setPower(1.0);
                rightServo.setPower(1.0);
            } else {
                leftServo.setPower(0.0);
                rightServo.setPower(0.0);
                movementCompleted = true;
                movementStarted = false;
                buttonPressed = false;
            }
        }
    }
}
