package org.firstinspires.ftc.team28420;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.module.Actions;
import org.firstinspires.ftc.team28420.module.Camera;
import org.firstinspires.ftc.team28420.module.Movement;
import org.firstinspires.ftc.team28420.module.Shooter;
import org.firstinspires.ftc.team28420.util.Config;

@TeleOp(name = "New Actions", group = "main")
public class NewActionsTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Config.Etc.telemetry = telemetry;

        Actions act = new Actions(
                new Movement(
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_TOP_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_TOP_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.LEFT_BOTTOM_MOTOR),
                        hardwareMap.get(DcMotorEx.class, Config.WheelBaseConf.RIGHT_BOTTOM_MOTOR)
                ),
                hardwareMap.get(IMU.class, Config.GyroConf.IMU),
                new Camera(hardwareMap.get(WebcamName.class, Config.CameraConf.WEBCAM)),
                new Shooter(hardwareMap),
                hardwareMap.get(Servo.class, "parkingServo")
        );

        act.init();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                act.move(act.getRatiosForApriltag());
            }
            else {
                act.move(
                        act.getRatios(
                                act.getCubic(act.withDeathzone(gamepad1.left_stick_x, Config.GamepadConf.LEFT_DEAD_ZONE)),
                                -1 * act.getCubic(act.withDeathzone(gamepad1.left_stick_y, Config.GamepadConf.LEFT_DEAD_ZONE)),
                                act.getCubic(act.withDeathzone(gamepad1.right_stick_x, Config.GamepadConf.RIGHT_DEAD_ZONE))
                        )
                );
            }

            act.updateShooter(gamepad2);

            if (gamepad1.dpad_up) {
                act.park();
            }

            act.log();

            telemetry.update();
        }
    }

}
