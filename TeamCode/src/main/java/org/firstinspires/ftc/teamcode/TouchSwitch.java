package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSwitch {

    private TouchSensor touchSw;
    public void init(HardwareMap hwMap, String device_name)
    {
        touchSw = hwMap.get(TouchSensor.class, device_name);
        //touchSw.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLimitSwitchPressed(){
        return touchSw.isPressed();
    }
}
