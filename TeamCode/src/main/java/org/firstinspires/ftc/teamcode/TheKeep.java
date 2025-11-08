package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="The Keep",group="The Keep")
public class TheKeep extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor shooter = null;
    private Servo ballEjector = null;
    private Servo spinIndexer = null;
    private double spinPosition = 0.6;
    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        ballEjector = hardwareMap.get(Servo.class, "ballEjector");
        spinIndexer = hardwareMap.get(Servo.class, "spinIndexer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        //These lines setup the motors and servos

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //These lines will set the direction of the motors so 1 power is forward

    } // This initializes all the variables and motors

    @Override
    public void loop() {

        leftDrive.setPower((gamepad1.left_stick_y-gamepad1.left_stick_x)/2);
        rightDrive.setPower((gamepad1.left_stick_y+gamepad1.left_stick_x)/2);
        // These lines control the wheel motors

        if (gamepad1.rightBumperWasPressed())  spinPosition = spinPosition + 0.2;
        if (gamepad1.leftBumperWasPressed()) spinPosition = spinPosition - 0.2;
        if (spinPosition>1) spinPosition = 1;
        if (spinPosition<0) spinPosition = 0;
        spinIndexer.setPosition(spinPosition);
        // These lines control the spin indexer

        if (gamepad1.right_stick_y>0) ballEjector.setPosition(.2);
        if (gamepad1.right_stick_y<=0) ballEjector.setPosition(0);

        // This controls the ball ejector

        if (gamepad1.dpad_up) shooter.setPower(1);
        if (gamepad1.dpad_down) shooter.setPower(0);
        // These lines control the shooting flywheel
        telemetry.addData("SpinIndexer Position", spinPosition);
        telemetry.update();

    } // This section controls all the motors using the remote control

}
