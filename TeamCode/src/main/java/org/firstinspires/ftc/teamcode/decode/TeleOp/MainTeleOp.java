package org.firstinspires.ftc.teamcode.decode.TeleOp;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.isRed;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo; // Added your subsystem
import com.pedropathing.geometry.Pose;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    // Hardware from TeleOp 2
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor flyWheelMotor, followerWheelMotor, intake, loader;
    private IMU imu;

    // Subsystems
    private Robot robot;
    private HoodServo hood = new HoodServo(); // Use your HoodServo class

    // Control variables
    private GamepadEx gp1;
    private PIDController pidController = new PIDController();
    public static PIDGains pidGains = new PIDGains(10, 0.0001, 0.000001);
    private boolean isAligning = false;
    private Pose goal = new Pose(136, 136);
    private double flyPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization (TeleOp 2 Style) ---
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");
        flyWheelMotor = hardwareMap.dcMotor.get("rightShooterMotor");
        followerWheelMotor = hardwareMap.dcMotor.get("leftShooterMotor");
        intake = hardwareMap.dcMotor.get("intakeMotor");
        loader = hardwareMap.dcMotor.get("loaderMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        followerWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Subsystem Initialization
        gp1 = new GamepadEx(gamepad1);
        robot = new Robot(hardwareMap);
        hood.init(hardwareMap); // Initialize your HoodServo subsystem

        pidController.setGains(pidGains);

        if (!isRed) goal = goal.mirror();

        // Default hood position on init
        hood.setHoodservo(0.45);

        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();
            robot.drivetrain.update(); // Keep odometry alive for auto-aim

            // --- 1. Driving Logic (Field Centric + Auto Align) ---
            double y = -gp1.getLeftY();
            double x = gp1.getLeftX();
            double rx = gp1.getRightX();

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) imu.resetYaw();

            // Trigger Auto-Align
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                pidController.setTarget(new State(Math.atan2(goal.getY() - robot.drivetrain.getPose().getY(), goal.getX() - robot.drivetrain.getPose().getX())));
                isAligning = true;
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (isAligning) {
                rx = pidController.calculate(new State(botHeading));
                if (pidController.isInTolerance(new State(botHeading), Math.toRadians(3))) isAligning = false;
            }

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX * 1.1) + Math.abs(rx), 1);

            frontLeft.setPower((rotY + rotX * 1.1 + rx) / denominator);
            backLeft.setPower((rotY - rotX * 1.1 + rx) / denominator);
            frontRight.setPower((rotY - rotX * 1.1 - rx) / denominator);
            backRight.setPower((rotY + rotX * 1.1 - rx) / denominator);

            // --- 2. Shooter Logic (TeleOp 2 Triggers) ---
            flyPower += gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.05;
            flyPower -= gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));

            flyWheelMotor.setPower(flyPower);
            followerWheelMotor.setPower(flyPower);

            // --- 3. Intake Logic (Right Bumper = 100%) ---
            if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPower(1.0);
            } else if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPower(-0.5);
            } else if (gp1.getButton(GamepadKeys.Button.DPAD_UP)) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }

            // --- 4. Loader Logic ---
            if (gp1.getButton(GamepadKeys.Button.Y) || gp1.getButton(GamepadKeys.Button.DPAD_UP)) {
                loader.setDirection(DcMotorSimple.Direction.FORWARD);
                loader.setPower(1.0);
            } else if (gp1.getButton(GamepadKeys.Button.A)) {
                loader.setPower(-1.0);
            } else {
                loader.setPower(0);
            }

            // --- 5. Hood Logic (Using your setHoodservo method) ---
            if (gp1.getButton(GamepadKeys.Button.X)) {
                hood.setHoodservo(0.45); // Close
            } else if (gp1.getButton(GamepadKeys.Button.B)) {
                hood.setHoodservo(0.2);  // Far
            } else if (gp1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                hood.setHoodservo(0.3);  // Top
            }

            telemetry.addData("Shooter Power", flyPower);
            telemetry.addData("Hood Position", hood.getPosition());
            telemetry.addData("Aligning", isAligning);
            telemetry.update();
        }
    }
}
//decrease p if overshoots
//increase p if it undershoots
//decrease d if it jitters to much
//increase d if it doesnt push p enough (not enough precision)
//increase i if there is lots of small error over TIME
//decrease i if it is higher than d or p, or jittering
//
//P SHOULD BE BIGGEST, d is smaller, and I is smallest
//
//tune p first, then d, then i
//
//start from 0.001 for p, 0.0001 for d, 0.000001 for i
//change 1 in decimal to 5, if not move up decimal place up; same if going down
