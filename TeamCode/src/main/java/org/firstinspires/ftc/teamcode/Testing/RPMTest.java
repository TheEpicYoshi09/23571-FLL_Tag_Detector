package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactTracker;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.Locale;

// <3.5 ft - 2,200 rpm
// 5-6ft(?) - 2,350 rpm
// max feet - 2,650 rpm

//@Disabled
@TeleOp(name = "RPM Test Mode", group = "TeleOp")
public class RPMTest extends LinearOpMode {
    private static final double TICKS_PER_REV = 28.0;

    RobotHardware robot = new RobotHardware(this);

    private boolean dpadUpPreviouslyPressed = false;
    private boolean dpadDownPreviouslyPressed = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private boolean dpadRightPreviouslyPressed = false;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2, Constants.spindexer3};

    private double targetRpm = 0.0;
    private double spinupSetpointRpm = 0.0;
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean measuringSpinup = false;

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private void setFlywheelRpm(double rpm) {
        if (rpm > 0 && targetRpm <= 0) {
            spinupSetpointRpm = rpm;
            spinupTimer.reset();
            measuringSpinup = true;
        }

        targetRpm = rpm;
        LauncherMotorGroup launcherGroup = robot.launcherGroup;
        if (launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        double ticksPerSecond = rpmToTicksPerSecond(targetRpm);
        launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherGroup.group.setVelocity(ticksPerSecond);
    }

    @Override
    public void runOpMode() {
        double oldTime = 0;

        boolean backButtonPreviouslyPressed = false;
        boolean rightBumperPreviouslyPressed = false;


        robot.init();

        int spindexerIndex = 0;
        robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
        robot.spindexerPos = spindexerPositions[spindexerIndex];

        TurretTracker turretTracker = new TurretTracker(robot, telemetry);
        FlywheelController flywheelController = new FlywheelController(robot, telemetry);
        ShootingController shootingController = new ShootingController(robot, flywheelController, telemetry);
        ArtifactTracker artifactTracker = new ArtifactTracker(robot, telemetry);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            robot.refreshLimelightResult();
            artifactTracker.update();

            //Limelight Data
            LLResult result = robot.getLatestLimelightResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx/ty", "tx: %.2f ty: %.2f", result.getTx(), result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }

            //Odometry
            robot.pinpoint.update(); //Update odometry
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            double VelX = robot.pinpoint.getVelX(DistanceUnit.MM);
            double VelY = robot.pinpoint.getVelY(DistanceUnit.MM);
            double headingVel = robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", VelX, VelY, headingVel);

            robot.updateHeadingOffsetFromAllianceButton();
            double botHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double adjustedHeading = robot.applyHeadingOffset(botHeading);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.FieldCentricDrive(x, y, rx, adjustedHeading);

            boolean trackingActive = flywheelController.isEnabled() || gamepad1.start;
            if (trackingActive) {
                turretTracker.update();
                robot.headlight.setPosition(Constants.headlightPower); //Set light power here
            } else {
                robot.turret.setPower(0);
                robot.headlight.setPosition(0.0);
            }

            // Flywheel toggle on gamepad1 back
            boolean backButtonPressed = gamepad1.back;
            if (backButtonPressed && !backButtonPreviouslyPressed) {
                flywheelController.toggle();
            }
            backButtonPreviouslyPressed = backButtonPressed;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            if (dpadUp && !dpadUpPreviouslyPressed) {
                setFlywheelRpm(targetRpm + 100);
            }

            if (dpadDown && !dpadDownPreviouslyPressed) {
                setFlywheelRpm(targetRpm - 100);
            }

            if (dpadRight && !dpadRightPreviouslyPressed) {
                setFlywheelRpm(targetRpm - 10);
            }

            if (dpadLeft && !dpadLeftPreviouslyPressed) {
                setFlywheelRpm(targetRpm + 10);
            }

            dpadUpPreviouslyPressed = dpadUp;
            dpadDownPreviouslyPressed = dpadDown;
            dpadLeftPreviouslyPressed = dpadLeft;
            dpadRightPreviouslyPressed = dpadRight;

            shootingController.update();

            if (shootingController.isIdle()) {
                //Manual Lift Control
                if (gamepad1.left_trigger >= 0.5) {
                    robot.kicker.setPosition(Constants.kickerUp);
                } else {
                    robot.kicker.setPosition(Constants.kickerDown);
                }

                //Spindexer Manual Control
                if (gamepad1.b) {
                    robot.spindexer.setPosition(Constants.spindexer1);
                    robot.spindexerPos = Constants.spindexer1;
                } else if (gamepad1.y) {
                    robot.spindexer.setPosition(Constants.spindexer2);
                    robot.spindexerPos = Constants.spindexer2;
                } else if (gamepad1.x) {
                    robot.spindexer.setPosition(Constants.spindexer3);
                    robot.spindexerPos = Constants.spindexer3;
                }
            }

            telemetry.addData("Flywheel RPM", "%.0f rpm", targetRpm);
            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
