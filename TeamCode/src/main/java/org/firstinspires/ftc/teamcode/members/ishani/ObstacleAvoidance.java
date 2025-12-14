/*
    Robot drives forward normally
    If distance sensor sees anything closer than 18 inches →
    → Turns 90° left → drives a bit → turns back → continues

 */

package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "IS - Obstacle Avoidance", group = "ISSensor")
public class ObstacleAvoidance extends LinearOpMode {

    // ———————— YOUR ROBOT PARTS ————————
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DistanceSensor frontSensor;   // ← your distance sensor pointing forward
    private IMU imu;

    // ———————— SETTINGS YOU ASKED FOR ————————
    private static final double DRIVE_POWER          = 0.2;   // ← super slow!
    private static final double MAX_TRAVEL_INCHES    = 60.0;  // ← 5 feet max
    private static final double SAFE_DISTANCE_INCHES = 18.0;  // turn if closer than this
    private static final double TURN_DEGREES         = 90.0;  // turn 90° when obstacle

    private double totalTraveled = 0.0;  // how far we have gone

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT MOTORS ————————
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ———————— 2. CONNECT DISTANCE SENSOR ————————
        frontSensor = hardwareMap.get(DistanceSensor.class, "distance");

        // ———————— 3. CONNECT IMU (helps us turn straight) ————————
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
        imu.resetYaw();

        telemetry.addData("OBSTACLE AVOIDANCE (No Robot Class)", "Power 0.2 | Max 5 feet");
        telemetry.addData("Safe distance", "%.0f inches", SAFE_DISTANCE_INCHES);
        telemetry.addData("Press PLAY → slow safe driving!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && totalTraveled < MAX_TRAVEL_INCHES) {

            double distance = frontSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance Ahead", "%.1f inches", distance);
            telemetry.addData("Total Traveled", "%.1f / 60 inches", totalTraveled);

            // ———————— TOO FAR ALREADY? STOP! ————————
            if (totalTraveled >= MAX_TRAVEL_INCHES) {
                stopDriving();
                telemetry.addData("MAX 5 FEET REACHED", "Stopping!");
                telemetry.update();
                break;
            }

            // ———————— OBSTACLE TOO CLOSE → TURN AWAY ————————
            if (distance < SAFE_DISTANCE_INCHES) {
                stopDriving();
                telemetry.addData("OBSTACLE!", "Turning 90°...");
                telemetry.update();

                turnDegrees(TURN_DEGREES);        // turn left 90°
                driveForwardInches(12, DRIVE_POWER);  // go forward a bit
                totalTraveled += 12;              // add to total
                turnDegrees(-TURN_DEGREES);       // turn back
            }

            // ———————— PATH CLEAR → DRIVE SLOWLY FORWARD ————————
            else {
                frontLeft.setPower(DRIVE_POWER);
                frontRight.setPower(DRIVE_POWER);
                backLeft.setPower(DRIVE_POWER);
                backRight.setPower(DRIVE_POWER);

                totalTraveled += 0.1;  // rough estimate at slow speed

                telemetry.addData("CLEAR", "Driving slowly forward");
            }

            telemetry.addData("Heading", "%.1f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            sleep(100);  // small pause
        }

        stopDriving();
        telemetry.addData("DONE", "Traveled %.1f inches safely!", totalTraveled);
        telemetry.update();
    }

    // ———————— HELPER: DRIVE FORWARD EXACT INCHES ————————
    private void driveForwardInches(double inches, double power) {
        // Simple time-based (good enough at slow speed)
        double timeSeconds = inches / (power * 10);  // rough guess: 0.2 power ≈ 2 inches/sec
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep((long)(timeSeconds * 1000));
        stopDriving();
    }

    // ———————— HELPER: TURN EXACT DEGREES (IMU) ————————
    private void turnDegrees(double degrees) {
        double target = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + degrees;
        double error = target - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (Math.abs(error) > 180) error -= 360 * Math.signum(error);

        while (Math.abs(error) > 2.0) {
            double turnPower = error > 0 ? 0.3 : -0.3;
            frontLeft.setPower(-turnPower);
            backLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);

            error = target - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            while (Math.abs(error) > 180) error -= 360 * Math.signum(error);
        }
        stopDriving();
    }

    // ———————— HELPER: STOP ALL MOTORS ————————
    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}