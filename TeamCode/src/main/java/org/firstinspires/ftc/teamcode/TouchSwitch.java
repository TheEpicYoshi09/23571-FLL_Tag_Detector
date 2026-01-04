package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

public class TouchSwitch {

    private TouchSensor touchSw;

    public void init(HardwareMap hwMap, String device_name)
    {
        touchSw = hwMap.get(TouchSensor.class, device_name);
    }

    public boolean isLimitSwitchPressed(){
        return touchSw.isPressed();
    }

}
