package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID.FlywheelPID;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

@TeleOp(name = "Impulse Drive With FlywheelPID")
public class DriveFlywheel extends OpMode {

    private double previousPid = 0.0;   // initialize PID contribution
    private double previousPower = 0.0; // for optional full motor ramp
    private boolean flywheelOn = false;

    private double targetRPM = 4000;
    private final double rpmStep = 50;
    private boolean upPressed = false;
    private boolean downPressed = false;

    private FlywheelPID pid;
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();

    private ElapsedTime timer = new ElapsedTime();
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;

    public DcMotorEx SIntake;
    private boolean xWasPressed = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelR");
        SIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0.0005, 0, 0.004);

        previousPid = 0.0;
        previousPower = 0.0;
        telemetry.addLine("Press dpad up/down to adjust RPM");
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !upPressed) {
            targetRPM += rpmStep;
            upPressed = true;
        }
        if (!gamepad1.dpad_up) {
            upPressed = false;
        }
        if (gamepad1.dpad_down && !downPressed) {
            targetRPM -= rpmStep;
            if (targetRPM < 0) targetRPM = 0;
            downPressed = true;
        }
        if (!gamepad1.dpad_down) {
            downPressed = false;
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();
    }


    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input3 = false;
        boolean input2 = gamepad1.x;

        drive.driveRobotRelative(y,x,turn);
        intake.NonStationary(input1);


        if (input2 && !xWasPressed) {
            flywheelOn = !flywheelOn;
        }
        xWasPressed = input2;

        double kF = flywheelOn ? 0.7 : 0.0;

        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;

        double pidOut = pid.calculate(targetRPM, avgRPM, 0.02);


        double rampedPid = previousPid + Math.signum(pidOut - previousPid) * 0.01;
        rampedPid = Math.max(-1, Math.min(1, rampedPid));

        double power = kF + rampedPid;

        if (!flywheelOn) power = 0;

        double minusTRPM = 200;

        if(avgRPM < targetRPM && avgRPM >(targetRPM-minusTRPM) && gamepad1.b){
            input3 = true;
        }
        else{
            input3 = false;
        }

        if(avgRPM <(targetRPM-minusTRPM) && gamepad1.b){
            SIntake.setPower(0);
        }

        intake.stationary(input3);
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);

        previousPid = rampedPid;

        telemetry.addData("Flywheel", flywheelOn ? "spinning :)" : "not spinning :(");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.addData("Power", power);
    }
}
