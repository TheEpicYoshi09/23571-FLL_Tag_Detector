package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "2025TeleOp_3231", group="Linear OpMode")
public class MecanumTeleOp_3231 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain2 driveTrain = new DriveTrain2(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        // ColorSensing colorSensor = new ColorSensing(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap);
        Blocker leftblocker = new Blocker(hardwareMap, telemetry, "left");
        Blocker rightblocker = new Blocker(hardwareMap, telemetry, "right");
        waitForStart();

        if (isStopRequested()) return;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeIsActive()) {

            // colorSensor.updateTelemetry();
            leftblocker.powerServo(currentGamepad2, telemetry);
            rightblocker.powerServo(currentGamepad2, telemetry);

            //The Y button is for shooting
            //Use the Dpad to change the speed of the motors
            boolean wasYPressed = gamepad2.y;
            boolean wasXPressed = gamepad2.x;
            boolean wasAPressed = gamepad2.a;
            boolean wasBPressed = gamepad2.b;

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Drivetrain code
            driveTrain.drive(currentGamepad1, telemetry);

            //Shooting Code
            if (wasXPressed) {
                // Allow Manual Shooting
                shooter.setMotorPower(shooter.getShooterPower());
            } else if (wasAPressed) {
                // Preset Values 50, 70, 100
                shooter.shooter50();
            } else if (wasBPressed) {
                //shooter.shooter70();
                shooter.randomShooter();
            } else if (wasYPressed) {
                shooter.shooter100();
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                shooter.reverseMotor();
            } else {
                shooter.setZero(shooter.getShooterMotor());
            }

            //Shooter power increases by 10% everytime Dpad up is pressed
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                shooter.increasePower();
            }

            //Shooter power decreases by 10% everytime Dpad down is pressed
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                shooter.decreasePower();
            }

            intake.run(currentGamepad1);
           // colorSensor.updateTelemetry();

            //Telemetry for movement motors and shooters
            telemetry.addData("motors", "frontLeft(%.2f) frontRight(%.2f) backLeft(%.2f) backRight(%.2f)", driveTrain.getFrontLeftPower(), driveTrain.getFrontRightPower(), driveTrain.getBackLeftPower(), driveTrain.getBackRightPower());
            telemetry.addData("shooter", "shooter(%.2f)", shooter.getShooterPower());
            telemetry.addData("shooter encoder", "shooter encoder (%.2f), ", shooter.shooterEncoderPosition());
            telemetry.addData("intake","intake (%.2f)", intake.getIntakePower());
            telemetry.addData("Left Blocker", "left blocker position (%.2f), ", leftblocker.getServoPosition());
            telemetry.addData("Right Blocker", "right blocker position (%.2f), ", rightblocker.getServoPosition());
            telemetry.update();
        }
    }
}
