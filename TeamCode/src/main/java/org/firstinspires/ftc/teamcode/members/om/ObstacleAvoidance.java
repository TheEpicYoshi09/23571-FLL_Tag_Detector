package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Om - Obstacle Avoidance")
public class ObstacleAvoidance extends LinearOpMode {

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DistanceSensor distance_sensor;

    IMU imu;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double DRIVE_POWER = 0.2;
    static final double TURN_POWER = 0.2;

    static final double OBSTACLE_THRESHOLD_INCHES = 14.0;

    static final int TURN_90_COUNTS = 950;

    static final double DETOUR_SIDE_INCHES = 18.0;

    static final double TOTAL_FORWARD_INCHES = 84.0;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        resetAllEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double distanceTraveledForward = 0.0;

        while (opModeIsActive() && distanceTraveledForward < TOTAL_FORWARD_INCHES) {

            double currentDistance = distance_sensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance", "%.1f in", currentDistance);
            telemetry.addData("Progress", "%.1f / 84.0 in", distanceTraveledForward);
            telemetry.update();

            if (currentDistance < OBSTACLE_THRESHOLD_INCHES && currentDistance > 0) {

                stopMotors();

                turnRight90();
                driveRobotRelative(DETOUR_SIDE_INCHES, 0.0, 0.0);
                turnLeft90();
                driveRobotRelative(DETOUR_SIDE_INCHES + 10, 0.0, 0.0);
                turnRight90();

                distanceTraveledForward += DETOUR_SIDE_INCHES;

                if (distanceTraveledForward > TOTAL_FORWARD_INCHES) {
                    distanceTraveledForward = TOTAL_FORWARD_INCHES;
                }

            } else {

                double remaining = TOTAL_FORWARD_INCHES - distanceTraveledForward;
                double chunk = Math.min(12.0, remaining);

                driveRobotRelative(chunk, 0.0, 0.0);
                distanceTraveledForward += chunk;
            }
        }

        stopMotors();

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    private void driveRobotRelative(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        if (maxPower == 0) maxPower = 1.0;

        frontLeftDrive.setPower(DRIVE_POWER * frontLeftPower / maxPower);
        frontRightDrive.setPower(DRIVE_POWER * frontRightPower / maxPower);
        backLeftDrive.setPower(DRIVE_POWER * backLeftPower / maxPower);
        backRightDrive.setPower(DRIVE_POWER * backRightPower / maxPower);

        int target = (int)(Math.abs(forward) * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + (int)(target * Math.signum(forward)));
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + (int)(target * Math.signum(forward)));
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + (int)(target * Math.signum(forward)));
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + (int)(target * Math.signum(forward)));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {}

        stopMotors();
        resetAllEncoders();
    }

    private void turnRight90() {

        int target = TURN_90_COUNTS;

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - target);

        frontLeftDrive.setPower(TURN_POWER);
        frontRightDrive.setPower(TURN_POWER);
        backLeftDrive.setPower(TURN_POWER);
        backRightDrive.setPower(TURN_POWER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {}

        stopMotors();
        resetAllEncoders();
    }

    private void turnLeft90() {

        int target = TURN_90_COUNTS;

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + target);

        frontLeftDrive.setPower(TURN_POWER);
        frontRightDrive.setPower(TURN_POWER);
        backLeftDrive.setPower(TURN_POWER);
        backRightDrive.setPower(TURN_POWER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {}

        stopMotors();
        resetAllEncoders();
    }

    private void resetAllEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}


//Mission: Support students in our community through STEM learning.
//Vision: A community where every student can succeed in STEM.