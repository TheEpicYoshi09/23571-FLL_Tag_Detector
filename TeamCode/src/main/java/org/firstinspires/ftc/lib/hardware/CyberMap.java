package org.firstinspires.ftc.lib.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CyberMap {
    public static HardwareMap hardwareMap;

    public static void instantiate(HardwareMap hardwareMap) {
        CyberMap.hardwareMap = hardwareMap;
    }
    private CyberMap(HardwareMap hardwareMap) {
        CyberMap.hardwareMap = hardwareMap;
    }
}
