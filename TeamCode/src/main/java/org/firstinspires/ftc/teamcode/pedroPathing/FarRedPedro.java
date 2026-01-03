package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;

import java.nio.file.Paths;

@Autonomous(name = "FarRedPedro", group = "Autonomous")
@Configurable // Panels
public class FarRedPedro extends LinearOpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    Chassis ch;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Shooter collectorBack;
    Shooter collectorFront;
    Servo flipper;
    GoalTagLimelight limelight;

    private int startDelay = 0;
    private int teamID;
    private boolean testingMode = false;
    private boolean shooting = false;;
    ElapsedTime timer = new ElapsedTime();


    public PathChain MOVETOLAUNCH;
    public PathChain PREPARETOCOLLECT1;
    public PathChain COLLECT11;
    public PathChain COLLECT12;
    public PathChain COLLECT13;
    public PathChain MOVETOLAUNCH2;
    public PathChain PREPARETOCOLLECT2;
    public PathChain COLLECT21;
    public PathChain COLLECT22;
    public PathChain COLLECT23;
    public PathChain MOVETOLAUNCH3;
    public PathChain ENDOFFLINE;

    private final Pose startPose = new Pose(81.850, 8.348, Math.toRadians(90));
    @Override
    public void runOpMode() {
        setPathState(0);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


        ch = new Chassis(hardwareMap);

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);

        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

        do {
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", Shooter.collectorPower);
            telemetry.update();

            if (gamepad1.bWasPressed()) {
                //goalTag.targetAprilTagID = 24;
                teamID = 24;
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                //goalTag.targetAprilTagID = 20;
                teamID = 20;
                GlobalStorage.setAlliance(20);
            } else if (gamepad1.yWasPressed()) {
                startDelay += 2;
            } else if (gamepad1.aWasPressed()) {
                startDelay -= 1;
            } else if (gamepad1.leftStickButtonWasPressed()) {
                testingMode = true;
            }

        } while (opModeInInit());

        while (opModeIsActive()) {
            //sleep(1000*startDelay);

            limelight.setTeam(teamID);

            collectorBack.setPower(Shooter.collectorPower);
            collectorFront.setPower(Shooter.collectorPower);

            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);

            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }

    public void buildPaths() {

            MOVETOLAUNCH = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.850, 8.348), new Pose(87.598, 19.324))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            PREPARETOCOLLECT1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.598, 19.324), new Pose(101.670, 35.838))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            COLLECT11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.670, 35.838), new Pose(108, 35.838))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            COLLECT12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108, 35.838), new Pose(113, 35.578))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            COLLECT13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(113, 35.578), new Pose(118, 35.578))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            MOVETOLAUNCH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 35.578), new Pose(87.776, 19.087))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            PREPARETOCOLLECT2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.776, 19.087), new Pose(102.189, 59.859))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            COLLECT21 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.189, 59.859), new Pose(109.331, 59.600))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            COLLECT22 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(109.331, 59.600), new Pose(115.044, 59.470))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            COLLECT23 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(115.044, 59.470), new Pose(120.887, 59.340))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            MOVETOLAUNCH3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.887, 59.340), new Pose(87.647, 19.087))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            ENDOFFLINE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.647, 19.087), new Pose(92.191, 26.229))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(70))
                    .build();

    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(MOVETOLAUNCH, Shooter.maxPower, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(PREPARETOCOLLECT1, Shooter.maxPower, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT11, Shooter.maxPower, true);
                    timer.reset();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    flipper.setPosition(1);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT12, Shooter.maxPower, true);
                        timer.reset();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    flipper.setPosition(0);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT13, Shooter.maxPower, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.followPath(MOVETOLAUNCH2, Shooter.maxPower,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(PREPARETOCOLLECT2, Shooter.maxPower, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT21, Shooter.maxPower, true);
                    timer.reset();
                    setPathState(8);
                }

            case 8:
                if (!follower.isBusy()) {
                    flipper.setPosition(1);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT22, Shooter.maxPower, true);
                        timer.reset();
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    flipper.setPosition(0);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT23, Shooter.maxPower, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOLAUNCH3, Shooter.maxPower, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(ENDOFFLINE, Shooter.maxPower, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }
    public void setPathState(int pState) {
        pathState = pState;
    }

}
