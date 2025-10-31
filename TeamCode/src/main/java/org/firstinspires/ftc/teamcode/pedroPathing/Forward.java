package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Forward extends OpMode {

    // Add Actuators
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor Intake;
    private Servo finger;
    private Timer actionTimer;
    private boolean shotFired = false;

    //
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define positions
    private final Pose startPose = new Pose(34.000, 135.500, Math.toRadians(-90));
    private final Pose Line1 = new Pose(48, 80, Math.toRadians(180));
    private final Pose Line2 = new Pose(25, 80, Math.toRadians(180));
    private final Pose shootpose = new Pose(48, 95.500, Math.toRadians(-45));
    private final Pose endPose = new Pose(34,80, Math.toRadians(180));
    private Path Path1;
    private Path Path2;
    private Path Path3;
    private Path Path4;
    private Path Leave;

    @Override
    public void init() {
        // Set up left shooter motor
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setPower(0);
        // Set up right shooter motor
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setPower(0);
        // Set up Intake motor
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setPower(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build a simple path between the two poses
        Path1 = new Path(new BezierLine(startPose, shootpose));
        Path1.setLinearHeadingInterpolation(startPose.getHeading(), shootpose.getHeading());

        Path2 = new Path(new BezierLine(shootpose, Line1));
        Path2.setLinearHeadingInterpolation(shootpose.getHeading(), Line1.getHeading());

        Path3 = new Path(new BezierLine(Line1, Line2));
        Path3.setTangentHeadingInterpolation();

        Path4 = new Path(new BezierLine(Line2, shootpose));
        Path4.setLinearHeadingInterpolation(Line2.getHeading(), shootpose.getHeading());

        Leave = new Path(new BezierLine(startPose, endPose));
        Leave.setTangentHeadingInterpolation();

        pathTimer = new Timer();
        pathState = 0;

        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        follower.followPath(Leave);
    }

    @Override
    public void loop() {
        follower.update();


/*
        if (!follower.isBusy()) {
            if (pathState == 0) {
                // Finished first move: start pause timer
                pathTimer.resetTimer();
                pathState = 1;
            } else if (pathState == 1) {
                // Check both time and position before continuing
                double dx = follower.getPose().getX() - shootpose.getX();
                double dy = follower.getPose().getY() - shootpose.getY();
                double distance = Math.hypot(dx, dy);

                if (pathTimer.getElapsedTimeSeconds() >= 5.0 && distance < 1.0) {
                    follower.followPath(Path2);
                    pathState = 2;
                }
            } else if (pathState == 2) {
                // Finished path2 (end -> Line1)
                double dx = follower.getPose().getX() - Line1.getX();
                double dy = follower.getPose().getY() - Line1.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    follower.followPath(Path3);
                    pathState = 3;
                }
            } else if (pathState == 3) {
                // Finished path3 (Line1 -> Line2)
                double dx = follower.getPose().getX() - Line2.getX();
                double dy = follower.getPose().getY() - Line2.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    follower.followPath(Path4);
                    pathState = 4;
                }
            } else if (pathState == 4) {
                // Finished returning to shootpose
                double dx = follower.getPose().getX() - shootpose.getX();
                double dy = follower.getPose().getY() - shootpose.getY();
                if (Math.hypot(dx, dy) < 1.0) {
                    pathState = 5; // Done
                    telemetry.addLine("All paths complete and robot in position");


                }
            }
        }

        telemetry.addData("segment", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading_deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

 */
    }
}