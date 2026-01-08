package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBlight {

    private Servo light;

    public void init(HardwareMap hwMap, String device_name)
    {
        light = hwMap.get(Servo.class, device_name);
    }

    public void light_on(double value)
    {
        light.setPosition(value);
    }
    public void light_off()
    {
        light.setPosition(0.0);
    }
}
