package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@Autonomous
public class testAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // --- Set up wheels ---
        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();

        // --- Set up vision ---
        // Set turret TX offset to +10 degrees → right
        robot.turret.setTxOffset(10);

        // Enable auto aim
        robot.turret.toggleAutoAim();
        robot.vision.clearAllowedTags();
        robot.vision.addAllowedTag(24);

        while (opModeIsActive()) {
            while (opModeIsActive()) {

            }
        }
    }
}

