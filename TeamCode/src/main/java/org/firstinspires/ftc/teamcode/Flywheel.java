package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flywheel {
    private static DcMotor FlywheelMotor;

    public static void init(HardwareMap hwMap) {
        FlywheelMotor = hwMap.get(DcMotor.class, "flywheel");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void spin(boolean input1, boolean input2) {

        if (input1) {
            FlywheelMotor.setPower(1);
        }
        if (input2) {
            FlywheelMotor.setPower(0);
        }
    }
}
