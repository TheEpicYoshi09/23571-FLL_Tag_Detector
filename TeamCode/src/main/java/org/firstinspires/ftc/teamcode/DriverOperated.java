package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.hardware.ActuatorConfig;
import org.firstinspires.ftc.lib.hardware.CyberDCM;

@TeleOp
public class DriverOperated extends OpMode {
    private MotorEx dcm;

    @Override
    public void init() {
        dcm = new MotorEx(hardwareMap, "1");
        dcm.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void loop() {

    }
}
