package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive 25-26", group="Opmode")
public class TeleOpMode extends OpMode
{
    //DONT DELETE
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double Kg = 0.07;
    private boolean boxUp = false;
    //private PIDController pid = new PIDController(0.06, 0, 0);


    //main thingies
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //wheels
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rightBack");


        //SETTINGS FOR MOTORS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //door.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //GAMEPAD 1 CONTROLS - Primary: Driving, Safety: Lift(Motors)
        //START DRIVING
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        frontLeftPower = (pivot+y+x);
        rearLeftPower = (pivot+y-x);
        frontRightPower = (-pivot+y-x);
        rearRightPower = (-pivot+y+x);


        // Driver control robot movements
        if(gamepad1.left_bumper) {
            // crawl mode
            frontLeft.setPower(frontLeftPower * 0.25);
            frontRight.setPower(frontRightPower * 0.25);
            rearLeft.setPower(rearLeftPower * 0.25);
            rearRight.setPower(rearRightPower * 0.25);
        }
        else {
            frontLeft.setPower(frontLeftPower * .9);
            frontRight.setPower(frontRightPower * .9);
            rearLeft.setPower(rearLeftPower * .9);
            rearRight.setPower(rearRightPower * .9);
        }


        /*if (gamepad1.x) {
            flipTClaw.setPosition(0);
        } */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

