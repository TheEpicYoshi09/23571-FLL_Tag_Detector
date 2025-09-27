package org.firstinspires.ftc.teamcode.auto;

public class AutoBlue {


package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

    @Autonomous(name = "Example Auto", group = "Examples")
    public class ExampleAuto extends OpMode {

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        private int pathState;

        private final Pose startPose = new Pose(63.4, 9, Math.toRadians(180)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(58, 90, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose align1Pose = new Pose(41.5, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
        private final Pose pickup1Pose = new Pose(26.5, 84, Math.toRadians(0));
        private final Pose align2Pose = new Pose(41.5, 60, Math.toRadians(0)); Middle (Second Set) of Artifacts from the Spike Mark.
        private final Pose pickup2Pose = new Pose(26.5, 60, Math.toRadians(0));
        private final Pose align1Pose = new Pose(41.5, 36, Math.toRadians(0)); // Lowest (Last Set) of Artifacts from the Spike Mark.
        private final Pose pickup2Pose = new Pose(26.5, 36, Math.toRadians(0));

        private Path scorePreload;
        private PathChain alignPickup1, grabPickup1, scorePickup1, alignPickup2, grabPickup2, scorePickup2, alignPickup3, grabPickup3, scorePickup3;

        public void buildPaths() {
            scorePreload = new Path(new BezierLine(startPose, scorePose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .build();

            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                    .build();
        }

    }
