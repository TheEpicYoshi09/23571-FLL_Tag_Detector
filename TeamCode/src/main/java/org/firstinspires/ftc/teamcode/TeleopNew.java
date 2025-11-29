package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp
public class TeleopNew extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    private float flyWheelVelocity = 1300;
    private float  ticksPerRev = 288;
    private float offset = 0;

    // --- NEW AND CHANGED VARIABLES ---
    // Motor power runs from -1.0 to 1.0. We use 'double' for precision.
    private double feedRollerSpeed = 0.0;
    // This is the change amount: 0.05 is 5% power change.
    private final double FEED_ROLLER_INCREMENT = 0.05;
    private final double MAX_FEED_ROLLER_SPEED = 1.0;
    // --- END OF NEW VARIABLES ---

    private boolean flyWheelPowered;
    private boolean agitatorPowered;


    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        agitator = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feedRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);

        // --- UPDATED TELEMETRY ---
        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("RIGHT/LEFT BUMPERS to change feed roller speed");
        telemetry.addLine("y to turn off all and reset feed roller");
        telemetry.update();
    }

    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();

        telemetry.addLine("Encoder Position: " + String.valueOf(feedRoller.getCurrentPosition()));
        telemetry.addLine("Offset: " + offset);

        // --- NEW TELEMETRY ---
        telemetry.addLine("Feed Roller Speed: " + String.format("%.2f", feedRollerSpeed));
        telemetry.update();
    }
    public double getLowestVoltage() {
        double lowestValue = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor) {
            if(sensor.getVoltage() < lowestValue && sensor.getVoltage() > 0.1) {
                lowestValue = sensor.getVoltage();
            }
        }
        if(lowestValue == Double.POSITIVE_INFINITY) {
            lowestValue = 14;
        }
        telemetry.addLine("Voltage: " + lowestValue + "V");
        return lowestValue;
    }
    public void basicMovement() {
        float x;
        float y;

        x = gamepad1.right_stick_x;
        y = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    public void turnOnMotors() {
        // --- Flywheel (A button) ---
        if(gamepad1.aWasPressed()) {
            flyWheelPowered = !flyWheelPowered;
        }

        // --- Agitator (B button) ---
        if(gamepad1.bWasPressed()) {
            if(agitatorPowered) {
                agitatorPowered = false;
                agitator.setPower(0);
            } else {
                agitatorPowered = true;
                agitator.setPower(1);
            }
        }

        // --- Feed Roller Speed Control (Bumpers) ---

        // ** FIX APPLIED HERE: Using the camel-cased function names that align with aWasPressed() **
        // Right bumper increases speed
        if (gamepad1.rightBumperWasPressed()) {
            feedRollerSpeed += FEED_ROLLER_INCREMENT;
            // Left bumper decreases speed
        } else if (gamepad1.leftBumperWasPressed()) {
            feedRollerSpeed -= FEED_ROLLER_INCREMENT;
        }

        // Clamp the speed to make sure it stays between -1.0 (full reverse) and 1.0 (full forward)
        feedRollerSpeed = Math.max(-MAX_FEED_ROLLER_SPEED, Math.min(MAX_FEED_ROLLER_SPEED, feedRollerSpeed));


        // --- All Stop / Reset (Y button) ---
        if(gamepad1.yWasPressed()) {
            float angleOff = (feedRoller.getCurrentPosition() % ticksPerRev);

            feedRollerSpeed = 0.0; // Reset our speed variable to 0

            feedRoller.setTargetPosition((int)(feedRoller.getCurrentPosition() - angleOff));
            feedRoller.setPower(1.0);
            feedRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            flywheel.setPower(0);
            agitator.setPower(0);
        }

        // --- Apply Power to Feed Roller ---
        if (!feedRoller.isBusy()) {
            // If the motor is NOT busy running to a position...
            // ...set it to the speed we chose with the bumpers.
            feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            feedRoller.setPower(feedRollerSpeed);
        }
    }

    public void flyWheel() {
        if(flyWheelPowered) {
            double multiplier = 14 / getLowestVoltage();
            telemetry.addData("Velocity", flyWheelVelocity * multiplier);
            flywheel.setVelocity(flyWheelVelocity * multiplier);
        } else {
            flywheel.setVelocity(0);
        }
    }
}