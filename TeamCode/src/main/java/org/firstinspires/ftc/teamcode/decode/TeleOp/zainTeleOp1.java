package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp(name = "Maryam failing Ap Eng")
public class zainTeleOp1 extends LinearOpMode {

    private IMU imu;

    // Subsystems
    private HoodServo hoodServo;

    // GamepadEx
    private GamepadEx gamepadEx1;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------- Motors --------
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor flyWheelMotor = hardwareMap.dcMotor.get("rightShooterMotor");
        DcMotor followerWheelMotor = hardwareMap.dcMotor.get("leftShooterMotor");

        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");

        // -------- Subsystem Init --------
        hoodServo = new HoodServo();
        hoodServo.init(hardwareMap);
        hoodServo.setHoodservo(0); // starting position

        // -------- GamepadEx --------
        gamepadEx1 = new GamepadEx(gamepad1);

        // -------- Motor Directions --------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------- IMU --------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        double flyPower = 0;

        waitForStart();

        while (opModeIsActive()) {

            gamepadEx1.readButtons();

            // -------- Field-Centric Mecanum --------
            double y = -gamepadEx1.getLeftY();
            double x = gamepadEx1.getLeftX();
            double rx = gamepadEx1.getRightX();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeft.setPower((rotY + rotX + rx) / denominator);
            backLeft.setPower((rotY - rotX + rx) / denominator);
            frontRight.setPower((rotY - rotX - rx) / denominator);
            backRight.setPower((rotY + rotX - rx) / denominator);

            // -------- Shooter --------
            flyPower += gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.05;
            flyPower -= gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));

            flyWheelMotor.setPower(flyPower);
            followerWheelMotor.setPower(flyPower);

            // -------- Intake --------
            if (gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPower(1);
            } else if (gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // -------- Loader --------
            if (gamepadEx1.getButton(GamepadKeys.Button.Y)) {
                loader.setPower(1);
            } else if (gamepadEx1.getButton(GamepadKeys.Button.A)) {
                loader.setPower(-1);
            } else if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) {
                loader.setPower(1);
                intake.setPower(1);
            } else {
                loader.setPower(0);
            }

            // -------- Hood (2 Servos) --------
            if (gamepadEx1.getButton(GamepadKeys.Button.X)) {
                hoodServo.setHoodservo(0.4);
            } else if (gamepadEx1.getButton(GamepadKeys.Button.B)) {
                hoodServo.setHoodservo(0.45);
            } else if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                hoodServo.setHoodservo(0.5);
            }

            // -------- Telemetry --------
            telemetry.addData("Shooter Power", flyPower);
            telemetry.addData("Hood Position", hoodServo.getPosition());
            telemetry.update();
        }
    }
}
