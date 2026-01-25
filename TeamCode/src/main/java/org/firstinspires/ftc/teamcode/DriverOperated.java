package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.hardware.CyberMap;

@TeleOp
public class DriverOperated extends OpMode {

    @Override
    public void init() {
        new CyberMap(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
