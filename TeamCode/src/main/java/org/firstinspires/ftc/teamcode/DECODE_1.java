package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.ORANGE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

@TeleOp(name = "Decode_1", group = "V7")
public class DECODE_1 extends OpMode {

    private double shooterPower = -0.4; // default power, close zone
    private final double TARGET_DISTANCE_IN = 27;  // desired distance
    private final double MAX_POWER = 0.3;
    private final double DISTANCE_TOLERANCE = 1.0;

    private RevBlinkinLedDriver blinkin;
    private DcMotorEx lfm, lbm, rfm, rbm;
    private DcMotorEx intake, sr, sl, lift;
    private boolean liftActivate = false;
    private CRServo tl, bl;
    private Servo micro;
    private long lastRumbleTime = 0;
    private final long rumbleCooldown = 500;
    private final ElapsedTime runtime = new ElapsedTime();
    private final double HALF_TIME = 100.0;
    private boolean secondHalf = false;

    private Limelight3A limelight;
    private MecanumDrive drive; // Pinpoint drive

    private final double CAMERA_HEIGHT_IN = 14.3;  // inches from floor
    private final double TARGET_HEIGHT_IN = 29.125;  // inches from floor (tag center)
    private final double CAMERA_ANGLE_DEG = 20.0;
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        lfm = hardwareMap.get(DcMotorEx.class, "lfm");
        lbm = hardwareMap.get(DcMotorEx.class, "lbm");
        rfm = hardwareMap.get(DcMotorEx.class, "rfm");
        rbm = hardwareMap.get(DcMotorEx.class, "rbm");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        sr = hardwareMap.get(DcMotorEx.class, "sr");
        sr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sl = hardwareMap.get(DcMotorEx.class, "sl");

        tl = hardwareMap.get(CRServo.class, "tl");
        bl = hardwareMap.get(CRServo.class, "bl");
        micro = hardwareMap.get(Servo.class, "micro");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lfm.setDirection(DcMotorSimple.Direction.REVERSE);
        lbm.setDirection(DcMotorSimple.Direction.REVERSE);
        rfm.setDirection(DcMotorSimple.Direction.FORWARD);
        rbm.setDirection(DcMotorSimple.Direction.FORWARD);
        sl.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        for (DcMotorEx motor : new DcMotorEx[]{lfm, lbm, rfm, rbm, intake, sr, sl}) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        // Initialize Pinpoint drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveTask();
        intakeTask();
        launcherTask();
        servoTask();
        liftTask();
        microTask();
        cameraTask(); // example tag coordinates
        readyToRUMBLE();
        rpmTelemetryTask();

        double s1VelocityTicksPerSec = sr.getVelocity();
        double s1RPM = (s1VelocityTicksPerSec / TICKS_PER_REV) * 60.0;

        telemetry.addData("Lift Active", liftActivate);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("S1 RPM", String.format("%.1f", s1RPM));
        telemetry.update();
    }

    private void rpmTelemetryTask() {
        double ticksPerSec = sr.getVelocity();
        double rpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

        if (rpm <= -2914) {
            long now = System.currentTimeMillis();
            if (now - lastRumbleTime > rumbleCooldown) {
                gamepad1.rumble(500);
                blinkin.setPattern(GREEN);
                lastRumbleTime = now;
            } else {
                blinkin.setPattern(RAINBOW_WITH_GLITTER);
            }
        }
    }
    private void driveTask() {
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        //Deadzone and directional smoothing
        double deadzone = 0.05;
        if (Math.abs(drive) < deadzone) drive = 0;
        if (Math.abs(strafe) < deadzone) strafe = 0;
        if (Math.abs(turn) < deadzone) turn = 0;

        //Directional "snapping" helps make straight down/up easier b/c sensitive sticks
        double magnitude = Math.hypot(strafe, drive);
        double angle = Math.atan2(drive, strafe); //In radians

        //If you're within ~10° of a direction, it should snap to it. (This is not working that well...😒)
        double snapThreshold = Math.toRadians(10);
        if (magnitude > 0.2) {
            if (Math.abs(angle - Math.PI / 2) < snapThreshold) { //Up
                strafe = 0;
                drive = magnitude;
            } else if (Math.abs(angle + Math.PI / 2) < snapThreshold) { //Down
                strafe = 0;
                drive = -magnitude;
            } else if (Math.abs(angle) < snapThreshold) { //Right
                strafe = magnitude;
                drive = 0;
            } else if (Math.abs(Math.abs(angle) - Math.PI) < snapThreshold) { //Left
                strafe = -magnitude;
                drive = 0;
            }
        }

        double lfPower = drive + strafe + turn;
        double lbPower = drive - strafe + turn;
        double rfPower = drive - strafe - turn;
        double rbPower = drive + strafe - turn;

        double max = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower),
                Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1.0) {
            lfPower /= max;
            lbPower /= max;
            rfPower /= max;
            rbPower /= max;
        }

        lfm.setPower(lfPower);
        lbm.setPower(lbPower);
        rfm.setPower(rfPower);
        rbm.setPower(rbPower);
    }
    private void intakeTask() {
        if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
        } else if (gamepad1.left_bumper) {
            intake.setPower(0.5);
        } else {
            intake.setPower(0);
        }
    }

    private void launcherTask() {
        if (gamepad1.dpad_up) shooterPower = -0.4;
        else if (gamepad1.dpad_right) shooterPower = -1;
        else if (gamepad1.dpad_down) shooterPower = -0.45;
        else if (gamepad1.dpad_left) shooterPower = -0.5;

        if (gamepad1.right_trigger > 0.1) {
            sr.setPower(shooterPower);
            sl.setPower(shooterPower);
        } else {
            sr.setPower(0);
            sl.setPower(0);
        }
    }

    private void servoTask() {
        if (gamepad1.x) {
            tl.setPower(-1.0);
            bl.setPower(1.0);
        } else if (gamepad1.a) {
            tl.setPower(1.0);
            bl.setPower(-1.0);
        } else {
            tl.setPower(0);
            bl.setPower(0);
        }
    }
    private void liftTask() {
        if (gamepad1.touchpad && !liftActivate) {
            liftActivate = true;
            lift.setPower(1.0);
        }

        if (gamepad1.triangle && liftActivate) {
            liftActivate = false;
            lift.setPower(0);
        }
    }
    private void microTask() {
        if (gamepad1.left_stick_button) micro.setPosition(0.7);
    }
    public void cameraTask() {
        LLResult result = limelight.getLatestResult();

        if (gamepad1.left_trigger > 0.1 && result != null && result.isValid()) {

            // Limelight vertical offset in degrees
            double ty = result.getTy();

            // Convert to radians
            double angleToTargetRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);

            // Compute forward distance
            double distanceInches = (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTargetRad);

            double error = distanceInches - TARGET_DISTANCE_IN;

            if (Math.abs(error) <= DISTANCE_TOLERANCE) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                blinkin.setPattern(GREEN);
                return;
            }

            double forwardPower = error * 0.02;
            forwardPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, forwardPower));

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardPower, 0), 0));

            blinkin.setPattern(RED);

        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            blinkin.setPattern(RAINBOW_RAINBOW_PALETTE);
        }
        blinkin.setPattern(RAINBOW_RAINBOW_PALETTE);
    }



    private void readyToRUMBLE() {
        telemetry.addData(">", "Are we RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no");
        if (!secondHalf && runtime.seconds() > HALF_TIME) {
            gamepad1.rumble(1000);
            blinkin.setPattern(ORANGE);
            secondHalf = true;
        }
    }
}
