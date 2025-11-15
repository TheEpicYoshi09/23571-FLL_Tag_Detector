package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//test
@TeleOp(name = "DistanceTest", group = "Sensor")
@Disabled
public class DistanceYawTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\nYaw: %.1f\n",
                    robot.distanceR.getDistance(DistanceUnit.INCH),
                    robot.distanceL.getDistance(DistanceUnit.INCH),
                    robot.getCurrentYaw()));
            telemetry.update();
        }
    }
}

