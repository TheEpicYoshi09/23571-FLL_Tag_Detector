package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareMapConfig {

    public DcMotorEx wheel_0, wheel_1, wheel_2, wheel_3;

    public DcMotorEx shooter_motor_0, shooter_motor_1;
    public DcMotorEx intake_motor;

    public Servo angle_servo;
    // public Servo servo_1;

    public WebcamName webcam;

    public HardwareMapConfig(HardwareMap hw) {

        wheel_0 = hw.get(DcMotorEx.class, "wheel_0");
        wheel_1 = hw.get(DcMotorEx.class, "wheel_1");
        wheel_2 = hw.get(DcMotorEx.class, "wheel_2");
        wheel_3 = hw.get(DcMotorEx.class, "wheel_3");

        shooter_motor_0 = hw.get(DcMotorEx.class, "shooter_motor_0");
        shooter_motor_1 = hw.get(DcMotorEx.class, "shooter_motor_1");

        intake_motor = hw.get(DcMotorEx.class, "intake_motor");

        angle_servo = hw.get(Servo.class, "angle_servo");
        // servo_1 = hw.get(Servo.class, "servo_1");

        webcam = hw.get(WebcamName.class, "webcam");
    }
}
