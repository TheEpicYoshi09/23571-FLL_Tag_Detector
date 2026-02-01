package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp1 w/servo from zain")
public class zainTeleOp1 extends LinearOpMode {
    private IMU imu;
    public Servo servo;
    private GamepadEx gp1; // FTCLib Gamepad Wrapper

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize GamepadEx
        gp1 = new GamepadEx(gamepad1);

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor flyWheelMotor = hardwareMap.dcMotor.get("rightShooterMotor");
        DcMotor followerWheelMotor = hardwareMap.dcMotor.get("leftShooterMotor");
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");

        servo = hardwareMap.get(Servo.class, "hoodServo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        double flyPower = 0;
        servo.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // IMPORTANT: Read the gamepad state at the start of every loop
            gp1.readButtons();

            // --- mecanum Drive Control ---
            // GamepadEx flips the Y axis automatically so up is positive
            double y = gp1.getLeftY();
            double x = gp1.getLeftX();
            double rx = gp1.getRightX();

            // Stick button to reset IMU
            if (gp1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate movement for field centric
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Triggers for shooter power
            flyPower += gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.05;
            flyPower -= gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));

            flyWheelMotor.setPower(flyPower);
            followerWheelMotor.setPower(flyPower);

            // Bumpers for intake
            if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPower(1);
            } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // Loader and shared intake control
            if (gp1.isDown(GamepadKeys.Button.Y)) {
                loader.setDirection(DcMotorSimple.Direction.FORWARD);
                loader.setPower(1);
            } else if (gp1.isDown(GamepadKeys.Button.A)) {
                loader.setPower(-1);
            } else if(gp1.isDown(GamepadKeys.Button.DPAD_UP)){
                loader.setPower(1);
                intake.setPower(1);
            } else {
                loader.setPower(0);
            }

            // Servo positions using Buttons and D-Pad
            if (gp1.isDown(GamepadKeys.Button.X)){
                servo.setPosition(0.3);
            } else if (gp1.isDown(GamepadKeys.Button.B)){
                servo.setPosition(0.75);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_DOWN)){
                servo.setPosition(0.5);
            } else if (gp1.isDown(GamepadKeys.Button.DPAD_RIGHT)){
                servo.setPosition(0.8);
            }

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("shooter Power", flyWheelMotor.getPower());
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
        }
    }
}