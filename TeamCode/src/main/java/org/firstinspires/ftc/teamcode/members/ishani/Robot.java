package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ULTIMATE ROBOT CLASS v3 — FINAL VERSION (2025-2026)
 * Everything you will EVER need in ONE file:
 * - Perfect encoder drive + strafe
 * - Perfect IMU turns
 * - Distance sensor
 * - Telemetry everywhere
 * - Public getters + setDrivePower() → NO MORE PRIVATE ERRORS!
 *
 * HOW TO USE:
 * robot = new Robot(hardwareMap, telemetry);
 * robot.driveStraight(24);
 * robot.turn(90);
 * robot.setDrivePower(...) in TeleOp
 */
public class Robot {

    // ———————— ROBOT PARTS ————————
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private DistanceSensor distanceSensor;

    // ———————— TELEMETRY & TIMER ————————
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    // ———————— YOUR CALIBRATION NUMBERS — CHANGE THESE! ————————
    private static final double FORWARD_TICKS_PER_INCH = 48.7;
    private static final double STRAFE_TICKS_PER_INCH  = 52.3;

    // ———————— SPEEDS ————————
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED  = 0.5;

    // ———————— CONSTRUCTOR — connects everything ————————
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        telemetry.addData("ROBOT v3", "Initializing — the best one yet!");
        telemetry.update();

        // MOTORS
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        // DISTANCE SENSOR (change name if yours is different!)
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addData("ROBOT READY", "All systems go!");
        telemetry.addData("Distance Sensor", "Connected");
        telemetry.addData("Heading", "%.1f°", getHeading());
        telemetry.update();
    }

    // DRIVE STRAIGHT
    public void driveStraight(double inches) {
        int ticks = (int)(inches * FORWARD_TICKS_PER_INCH);
        timer.reset();

        resetEncoders();
        setTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition();
        setPowerAll(DRIVE_SPEED);

        telemetry.addData("DRIVE", "%.1f inches forward", inches);
        while (frontLeft.isBusy() && timer.seconds() < 8) {
            telemetry.addData("Progress", "%.1f / %.1f",
                    frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH, inches);
            telemetry.addData("Distance", "%.1f in", getDistanceInches());
            telemetry.update();
        }
        stopMotors();
        runUsingEncoders();
        telemetry.addData("DRIVE DONE", "%.1f inches", inches);
        telemetry.update();
        sleep(300);
    }

    // STRAFE
    public void strafe(double inches) {
        int ticks = (int)(Math.abs(inches) * STRAFE_TICKS_PER_INCH);
        timer.reset();

        resetEncoders();
        if (inches > 0) {
            setTargetPosition(ticks, -ticks, -ticks, ticks);  // right
        } else {
            setTargetPosition(-ticks, ticks, ticks, -ticks);  // left
        }
        runToPosition();
        setPowerAll(DRIVE_SPEED);

        telemetry.addData("STRAFE", inches > 0 ? "RIGHT →" : "LEFT ←");
        while (frontLeft.isBusy() && timer.seconds() < 8) {
            telemetry.addData("Progress", "%.1f in",
                    Math.abs(frontLeft.getCurrentPosition()) / STRAFE_TICKS_PER_INCH);
            telemetry.update();
        }
        stopMotors();
        runUsingEncoders();
        telemetry.addData("STRAFE DONE", "%.1f inches", Math.abs(inches));
        telemetry.update();
        sleep(300);
    }

    // TURN
    public void turn(double degrees) {
        double target = getHeading() + degrees;
        timer.reset();

        double error = target - getHeading();
        while (Math.abs(error) > 180) error -= 360 * Math.signum(error);

        telemetry.addData("TURN", degrees > 0 ? "LEFT %.0f°" : "RIGHT %.0f°", Math.abs(degrees));
        while (Math.abs(error) > 1.0 && timer.seconds() < 6) {
            double power = error > 0 ? TURN_SPEED : -TURN_SPEED;
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getHeading();
            while (Math.abs(error) > 180) error -= 360 * Math.signum(error);

            telemetry.addData("Current", "%.1f° | Error %.1f°", getHeading(), error);
            telemetry.update();
        }
        stopMotors();
        telemetry.addData("TURN DONE", "At %.1f°", getHeading());
        telemetry.update();
        sleep(300);
    }

    // STOP AT DISTANCE
    public void stopAtDistance(double targetInches) {
        telemetry.addData("STOP AT", "%.1f inches...", targetInches);
        while (getDistanceInches() > targetInches + 1.0) {
            double speed = 0.3;
            if (getDistanceInches() < targetInches + 8) speed = 0.2;
            setPowerAll(speed);
            telemetry.addData("Distance", "%.1f → %.1f", getDistanceInches(), targetInches);
            telemetry.update();
        }
        stopMotors();
        telemetry.addData("PERFECT STOP", "At %.1f inches", getDistanceInches());
        telemetry.update();
    }

    // MANUAL DRIVE FOR TELEOP — NO MORE PRIVATE ERROR!
    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // PUBLIC GETTERS — use these anywhere!
    public double getDistanceInches() { return distanceSensor.getDistance(DistanceUnit.INCH); }
    public double getHeading()        { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }

    // HELPER FUNCTIONS (private — you don't call these directly)
    private void setPowerAll(double power) {
        frontLeft.setPower(power);  frontRight.setPower(power);
        backLeft.setPower(power);   backRight.setPower(power);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTargetPosition(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(fl);   frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);    backRight.setTargetPosition(br);
    }

    private void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        setPowerAll(0);
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception e) {}
    }
}