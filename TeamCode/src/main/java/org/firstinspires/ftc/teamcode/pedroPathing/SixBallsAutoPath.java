package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class SixBallsAutoPath {

    public static PathChain build(Follower follower) {

        return follower.pathBuilder()

                // Path 1
                .addPath(new BezierLine(
                        new Pose(88.122, 7.597),
                        new Pose(87.784, 105.848)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))

                // Path 2
                .addPath(new BezierLine(
                        new Pose(87.784, 105.848),
                        new Pose(102.640, 118.846)))
                .setTangentHeadingInterpolation()

                // Path 3
                .addPath(new BezierLine(
                        new Pose(102.640, 118.846),
                        new Pose(95.043, 35.283)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                // Path 4
                .addPath(new BezierLine(
                        new Pose(95.043, 35.283),
                        new Pose(142.649, 35.451)))
                .setTangentHeadingInterpolation()

                // Path 5
                .addPath(new BezierLine(
                        new Pose(142.649, 35.451),
                        new Pose(95.212, 35.114)))
                .setTangentHeadingInterpolation()
                .setReversed()

                // Path 6
                .addPath(new BezierLine(
                        new Pose(95.212, 35.114),
                        new Pose(102.640, 118.846)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))

                // Path 7
                .addPath(new BezierLine(
                        new Pose(102.640, 118.846),
                        new Pose(102.640, 70.227)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                // Path 8
                .addPath(new BezierLine(
                        new Pose(102.640, 70.227),
                        new Pose(134.209, 70.059)))
                .setTangentHeadingInterpolation()

                // Path 9
                .addPath(new BezierLine(
                        new Pose(134.209, 70.059),
                        new Pose(102.640, 70.227)))
                .setTangentHeadingInterpolation()
                .setReversed()

                // Path 10
                .addPath(new BezierLine(
                        new Pose(102.640, 70.227),
                        new Pose(51.320, 33.088)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                // Path 11
                .addPath(new BezierLine(
                        new Pose(51.320, 33.088),
                        new Pose(40.009, 32.919)))
                .setTangentHeadingInterpolation()

                .build();
    }
}
