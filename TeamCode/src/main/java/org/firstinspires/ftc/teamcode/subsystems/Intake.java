package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private DcMotorEx intakeMotor;
    double maxVelocity;
    private boolean isIntakeRunning;

    public Intake() {
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        maxVelocity = 150;
    }

    public void setMaxVelocity(){
        intakeMotor.setVelocity(maxVelocity);
        isIntakeRunning = true;
    }

    public void setZeroVelocity(){
        intakeMotor.setVelocity(0);
    }


    public boolean getIsIntakeRunning(){
        return isIntakeRunning;
    }

}
