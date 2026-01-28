//Variable Hood Mecanum Teleop

package org.firstinspires.ftc.teamcode;
/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 * (standard license text unchanged)
 */


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "my1_27_2026CameraTeleop", group = "StarterBot")
//@Disabled
public class my1_27_2026CameraTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double FORWARD_SPEED = 0.5;
    final double MID_SPEED = 0.25;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double LAUNCHER_TARGET_VELOCITY = 1300;
    double LAUNCHER_MIN_VELOCITY = 1270;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo hood = null;

    ElapsedTime feederTimer = new ElapsedTime();

    // Data Logging Stuff - We'll make it store all the same stuff as telemetry
    //DataLog datalog;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void init() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        launchState = LaunchState.IDLE;

        leftFront = hardwareMap.get(DcMotor.class, "left_Front");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        hood = hardwareMap.get(Servo.class, "hood");

        // Motor directions
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        //leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        /*
         * Initialize the AprilTag processor.
         */
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();



        }




    @Override
    public void init_loop() {}

    @Override
    public void start() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)6, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(250);


    }

    @Override
    public void loop() {

        boolean targetFound = false;
        desiredTag  = null;



        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            //telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.update();
            double yawRadians = toRadians(desiredTag.ftcPose.yaw);
            double horizontalError = desiredTag.ftcPose.range * tan(yawRadians);

            if (abs(horizontalError) >= 2.61 && (desiredTag.id == 20 || desiredTag.id == 24)) {
                telemetry.addData("Clear to shoot", "false, please reposition robot");
                telemetry.update();

            }
            else if (abs(horizontalError) < 2.61) {
                telemetry.addData("Clear to shoot", "true, clear to shoot");
            }
        }





    // ⭐ FRONT/BACK FLIPPED ⭐
        // (Only line changed)
        double x = -gamepad1.left_stick_y;   // ← forward/back reversed
        double y =  gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        double leftFrontPower  = x + y + rotation;
        double rightFrontPower = x - y - rotation;
        double leftBackPower   = x - y + rotation;
        double rightBackPower  = x + y - rotation;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }
        if (gamepad1.left_bumper && gamepad1.left_trigger > 0) {
            hood.setPosition(0.45);
            LAUNCHER_TARGET_VELOCITY = 1325;
            LAUNCHER_MIN_VELOCITY = 1275;
        }   else if (gamepad1.left_bumper) {
            hood.setPosition(0.5);
            LAUNCHER_TARGET_VELOCITY = 1375;
            LAUNCHER_MIN_VELOCITY = 1325;
        }   else if (gamepad1.left_trigger > 0) {
            hood.setPosition(0.55);
            LAUNCHER_TARGET_VELOCITY = 1425;
            LAUNCHER_MIN_VELOCITY = 1375;
        }   else {
            hood.setPosition(0.45);
            LAUNCHER_TARGET_VELOCITY = 1325;
            LAUNCHER_MIN_VELOCITY = 1275;
        }

        launch(gamepad1.rightBumperWasPressed());

        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack (%.2f)",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() { }

    double softenedDrive(double x) {
        return 0.7 * pow(x, 3) + 0.3 * x;
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
