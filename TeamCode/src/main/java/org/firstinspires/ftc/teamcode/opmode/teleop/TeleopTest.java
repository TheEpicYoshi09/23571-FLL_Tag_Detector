package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

@TeleOp
@Disabled
public class TeleopTest extends LinearOpMode {

    /*
    private enum State {
        ARM_UP,
        ARM_DOWN,
        HANG_INITIALIZATION,
        HANG,
    }
    State state = State.ARM_DOWN;
*/
    private ElapsedTime runtime = new ElapsedTime();
    Servo intake = null;
    Servo tilt = null;
    Servo extent = null;
    Servo rotator = null;
    Servo grabber = null;
    Servo grabber_tilt = null;
    Servo sweeper = null;
    TouchSensor touch = null;

    /*
    public static double GRABBER_INIT = 0.5;
    public static double GRABBER_FORWARD = 0;
    public static double GRABBER_BACKWARD = 1;

    public static double GRABBER_ROTATOR_INIT = 0.43;
    public static double GRABBER_ROTATOR_DOWN = 0.38;
    public static double GRABBER_ROTATOR_UP = 0.44;

    public static double EXTENT_INIT = 0.9;
    public static double EXTENT_OUT = 0.3;
    public static double EXTENT_BACK = EXTENT_INIT;

    public static double TILT_INIT = 0.285;
    public static double TILT_DOWN = TILT_INIT;
    public static double TILT_UP = 0.55;

     */
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.00003;

    public static double f = 0.005;

    public static int target = 0;
    public boolean leftOpen = false;
    public boolean rightOpen = false;
    private final double ticks_in_degree = 700 / 180.0;

    private static final double STEP_INCHES = 1;
    private static final int TICKS_PER_INCH = 20;

    private DcMotor rightFly = null;
    private DcMotor leftFly = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private boolean sweeperExtent = false;
    private double sweeper_starttime = 0;
    private boolean tiltDown = false;
    private double tiltdown_starttime = 0;
    private boolean processSampleHighBasket = false;
    private double processSampleHighBasket_starttime = 0;
    private boolean processSampleLowBasket = false;
    private double processSampleLowBasket_starttime = 0;



    @Override
    public void runOpMode() throws InterruptedException {



        /*
        frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

         */

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        rightFly = hardwareMap.dcMotor.get("rightFly");
        rightFly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFly.setPower(0);

        leftFly = hardwareMap.dcMotor.get("leftFly");
        leftFly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFly.setPower(0);

        rightFly.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor slider = hardwareMap.dcMotor.get("slider");
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);

        DcMotor actuator = hardwareMap.dcMotor.get("actuator");
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setPower(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
        //telemetry.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
        telemetry.update();

        intake = hardwareMap.servo.get("intake");
        intake.setPosition(BotCoefficients.INTAKE_INIT);

        /*
        slider.setTargetPosition(-400);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
        sleep(1000);

         */
        tilt = hardwareMap.servo.get("tilt");
        //tilt.setPosition(BotCoefficients.TILT_INIT);
        /*
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));

         */

        extent = hardwareMap.servo.get("extent");
        extent.setPosition(BotCoefficients.EXTENT_INIT);

        rotator = hardwareMap.servo.get("rotator");
        rotator.setPosition(0.4);

        grabber = hardwareMap.servo.get("grabber");
        grabber.setPosition(BotCoefficients.GRABBER_INIT);

        grabber_tilt = hardwareMap.servo.get("grabber_tilt");
        grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_INIT);

        sweeper = hardwareMap.servo.get("sweeper");
        sweeper.setPosition(BotCoefficients.SWEEPER_INIT);

        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");

        /*
        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

        CRServo launchServo = hardwareMap.crservo.get("launcher");
        grabberTilt = hardwareMap.servo.get("grabberTilt");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL = hardwareMap.servo.get("grabberL");
        */

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);;
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);;

        controller = new PIDController(p, i, d);

        GamepadEx game1 = new GamepadEx(gamepad1);
        GamepadEx game2 = new GamepadEx(gamepad2);

        /*
        launchServo.setPower(0);
        grabberL.setPosition(BotCoefficients.grabberLeftClose);
        grabberR.setPosition(BotCoefficients.grabberRightClose);
        grabberTilt.setPosition(BotCoefficients.grabberUp);

         */

        waitForStart();


        while (!isStopRequested()) {

            /*
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x*0.8;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1.2);
            double frontLeftPower = (vertical - horizontal + turn) / denominator;
            double frontRightPower = (vertical + horizontal - turn) / denominator;
            double backLeftPower = (vertical + horizontal + turn) / denominator;
            double backRightPower = (vertical - horizontal - turn) / denominator;

            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

             */
            double horizontal = -1.0 * gamepad1.left_stick_x * 0.6;
            double vertical = gamepad1.left_stick_y * 0.6;
            double turn = -1.0 * gamepad1.right_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            setDrivePower(flPower, frPower, blPower, brPower);

            // control slider
            // x button for down
            if (gamepad2.x && !tiltDown){
                /*
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
                telemetry.update();

                int newSliderTarget = slider.getCurrentPosition() + 150;
                if (newSliderTarget <= BotCoefficients.SLIDER_BOTTOM_POSITION) {
                    slider.setTargetPosition(newSliderTarget);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));
                }

                 */
                /*
                if (tilt.getPosition() > 0.9) {

                    rotator.setPosition(0.39);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));
                }
                else {
                    tilt.setPosition(BotCoefficients.TILT_DOWN);
                }

                 */

                tilt.setPosition(BotCoefficients.TILT_DOWN);
                tiltDown = true;
                tiltdown_starttime = runtime.milliseconds();
            }
            // y button for up
            else if(gamepad2.y){

                /*
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
                telemetry.update();
                int newSliderTarget = slider.getCurrentPosition() - 120;
                if (newSliderTarget > BotCoefficients.SLIDER_TOP_POSITION) {
                    slider.setTargetPosition(newSliderTarget);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
                }

                 */
                rotator.setPosition(0.39);
                slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
                tilt.setPosition(0.55);
                processSampleHighBasket = true;
                processSampleHighBasket_starttime = runtime.milliseconds();
            }
            else {
                if ((slider.getCurrentPosition() > -2) || (touch.isPressed())){
                    slider.setPower(0);
                }
                else {
                    slider.setPower(BotCoefficients.SLIDER_HOLD_POWER);
                }
            }
            // put sample in the high basket, ready to come down
            /*
            if (processSampleHighBasket && (runtime.milliseconds() - processSampleHighBasket_starttime > 1000)) {
                tilt.setPosition(BotCoefficients.TILT_DOWN);
                tiltDown = true;
                tiltdown_starttime = runtime.milliseconds();
                processSampleHighBasket = false;
            }

             */
            if (tiltDown && (runtime.milliseconds() - tiltdown_starttime > 500)) {
                rotator.setPosition(0.39);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));
                tiltDown = false;
            }



            // Control actuator up and down
            if (gamepad1.y) {

                    actuator.setTargetPosition(BotCoefficients.ACTUATOR_TOP);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(Math.abs(0.8));
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
                    telemetry.update();


            }
            else if (gamepad1.x) {

                    actuator.setTargetPosition(0);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(Math.abs(0.8));
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
                    telemetry.update();


            }
            else {
                int currentPos = actuator.getCurrentPosition();
                if ((currentPos <= 0) || (currentPos>=BotCoefficients.ACTUATOR_TOP)) {
                    actuator.setPower(0);
                }

            }
            // control horizontal extention
            if (gamepad2.a) {
                extent.setPosition(BotCoefficients.EXTENT_BACK);
            }

            if (gamepad2.b) {
                extent.setPosition(0.6);
            }

            // Score Sample in the low backet

            if (gamepad1.a) {
                rotator.setPosition(0.39);
                slider.setTargetPosition(-80);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
                tilt.setPosition(BotCoefficients.TILT_UP);

                processSampleLowBasket = true;
                processSampleLowBasket_starttime = runtime.milliseconds();
            }

            if (processSampleLowBasket && (runtime.milliseconds() - processSampleLowBasket_starttime > 1000)) {
                tilt.setPosition(BotCoefficients.TILT_DOWN);
                tiltDown = true;
                tiltdown_starttime = runtime.milliseconds();
                processSampleLowBasket = false;
            }

            // Activate sweeper to push samples in the submersible
            if (gamepad1.b) {
                if (!sweeperExtent && (runtime.milliseconds()-sweeper_starttime > 500)) {
                    sweeper.setPosition(BotCoefficients.SWEEPER_EXTENT);
                    sweeperExtent = true;
                    sweeper_starttime = runtime.milliseconds();
                }
                else {
                    if (sweeperExtent && (runtime.milliseconds()-sweeper_starttime > 500)) {
                        sweeper.setPosition(BotCoefficients.SWEEPER_INIT);
                        sweeperExtent = false;
                        sweeper_starttime = runtime.milliseconds();
                    }
                }
            }
            /*
            if (sweeperExtent && (runtime.seconds()-sweeper_starttime > 1)) {
                sweeper.setPosition(BotCoefficients.SWEEPER_INIT);
                sweeperExtent = false;
            }

             */

            // control grabber
            if (gamepad1.right_bumper) {
                // open
                grabber.setPosition(BotCoefficients.GRABBER_OPEN);
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad1.right_trigger > 0.3){
                //close
                grabber.setPosition(BotCoefficients.GRABBER_CLOSE);
                //leftOpen = false;
            }
            // control grabber tilt
            if (gamepad1.left_bumper) {
                // open
                grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_UP);
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad1.left_trigger > 0.3){
                //close
                grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_DOWN);
                //leftOpen = false;
            }
            // control intake tilt
            if (gamepad2.left_bumper) {
                // open
                if (extent.getPosition() > 0.85) {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
                }
                else {
                    rotator.setPosition(0.4);
                }
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad2.left_trigger > 0.3){
                //close
                if (extent.getPosition() < 0.5) {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN_EXTENT);
                }
                else {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);
                }
                //leftOpen = false;
            }

            //control vertical bucket tilt
            if ( gamepad2.right_bumper) {
                // open
                //tilt.setPosition(BotCoefficients.tiltUp);
                tilt.setPosition(0.6);
            }
            if  (gamepad2.right_trigger > 0.3){
                //close
                //tilt.setPosition(BotCoefficients.tiltDown);
                tilt.setPosition(BotCoefficients.TILT_DOWN);
            }

            // for testing purpose

            if (gamepad1.dpad_up){
                encoderDrive(0.2, 2.5, 2.5, 3000);

                rightFly.setTargetPosition(BotCoefficients.ELBOW_READY);
                rightFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFly.setPower(0.2);

                leftFly.setTargetPosition(BotCoefficients.ELBOW_READY);
                leftFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftFly.setPower(0.2);

                rightFly.setTargetPosition(BotCoefficients.ELBOW_LIFT);
                rightFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFly.setPower(1);

                leftFly.setTargetPosition(BotCoefficients.ELBOW_LIFT);
                leftFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftFly.setPower(1);
                sleep(2000);
                encoderDrive(0.4, 50, 50, 10000);
            }

            if (gamepad1.dpad_down) {
                rightFly.setTargetPosition(0);
                rightFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFly.setPower(0.2);

                leftFly.setTargetPosition(0);
                leftFly.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftFly.setPower(0.2);
            }


            //control grabber
            if (gamepad2.dpad_up) {
                intake.setPosition(BotCoefficients.INTAKE_FORWARD);
            }
            if (gamepad2.dpad_down) {
                intake.setPosition(BotCoefficients.INTAKE_BACKWARD);
            }
            if (gamepad2.dpad_right || gamepad2.dpad_left ) {
                intake.setPosition(BotCoefficients.INTAKE_INIT);
            }
        }

    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * 39.79);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(rightInches * 39.79);
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * 39.79);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * 39.79);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


}