package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TouchSwitch {

    private DigitalChannel touchSw;
    public void init(HardwareMap hwMap, String device_name)
    {
        touchSw = hwMap.get(DigitalChannel.class, device_name);
        touchSw.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLimitSwitchPressed(){
        return !touchSw.getState();
    }
}
