package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ActiveOpMode {
    public static final ActiveOpMode INSTANCE = new ActiveOpMode();
    private ActiveOpMode() { }

    public LinearOpMode activeOpMode = null;
    public void init(LinearOpMode opMode) {
        activeOpMode = opMode;
    }
    public boolean isActive() {
        return activeOpMode.opModeIsActive();
    }
}
