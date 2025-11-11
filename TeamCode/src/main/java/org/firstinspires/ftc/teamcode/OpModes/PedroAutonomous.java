package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ToGoal;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            ToGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 124.000), new Pose(30.000, 124.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30.000, 124.000),
                                    new Pose(74.000, 84.000),
                                    new Pose(42.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 84.000), new Pose(18.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.000), new Pose(30.000, 124.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30.000, 124.000),
                                    new Pose(74.000, 60.000),
                                    new Pose(42.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 60.000), new Pose(18.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 60.000), new Pose(30.000, 124.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30.000, 124.000),
                                    new Pose(74.000, 35.000),
                                    new Pose(42.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 35.000), new Pose(18.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 35.000), new Pose(30.000, 124.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.000, 124.000), new Pose(24.000, 80.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
