package org.firstinspires.ftc.teamcode.decode.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    private final Follower f;

    // -----------------------------
    // PATHS
    // -----------------------------

    public PathChain shootPreload;
    public PathChain intake3;
    public PathChain shoot3;
    public PathChain intake6;
    public PathChain shoot6;
    public PathChain unloadRamp;
    public PathChain intake9;
    public PathChain shoot9;
    public PathChain intakeExtra1;
    public PathChain shootExtra1;
    public PathChain intakeExtra2;
    public PathChain shootExtra2;
    public PathChain intakeExtra3;
    public PathChain shootExtra3;
    public PathChain leave;

    // -----------------------------
    // ALLIANCE
    // -----------------------------


    public static boolean IS_RED = true;

    // -----------------------------
    // POSES
    // -----------------------------

    public static Pose P_START          = new Pose(112.094, 138.091);
    public static Pose P_PRELOAD_END    = new Pose(89.000, 102.000);
    public static Pose P_AUDIENCE_SHOT  = new Pose(89.000, 102.000);

    public static Pose P_I3_CP   = new Pose(79.512, 88.291);
    public static Pose P_I3_END  = new Pose(95.550, 83.057);
    public static Pose P_I3_WALL = new Pose(128.975, 83.395);

    public static Pose P_I6_CP   = new Pose(73.435, 79.006);
    public static Pose P_I6_END  = new Pose(96.563, 60.098);
    public static Pose P_I6_WALL = new Pose(136.066, 59.423);

    public static Pose P_RAMP_CP     = new Pose(92.680, 75.123);
    public static Pose P_RAMP_END    = new Pose(128.807, 70.227);
    public static Pose P_RAMP_WALL   = new Pose(136.066, 70.227);
    public static Pose P_RAMP_RETURN = new Pose(128.807, 70.227);

    public static Pose P_I9_CP     = new Pose(99.939, 61.618);
    public static Pose P_I9_END    = new Pose(99.264, 35.451);
    public static Pose P_I9_WALL   = new Pose(136.066, 35.451);
    public static Pose P_I9_RETURN = new Pose(99.433, 35.451);

    public static Pose P_EX_CP1  = new Pose(93.862, 66.851);
    public static Pose P_EX_END1 = new Pose(133.027, 60.774);

    public static Pose P_EX_CP2  = new Pose(93.862, 66.682);
    public static Pose P_EX_END2 = new Pose(133.027, 60.774);

    public static Pose P_EX_CP3  = new Pose(94.368, 66.513);
    public static Pose P_EX_END3 = new Pose(133.027, 60.605);

    public static Pose P_LEAVE_END = new Pose(89.135, 52.164);

    // -----------------------------
    // HEADINGS
    // -----------------------------

    public static double H_90  = Math.toRadians(90);
    public static double H_114 = Math.toRadians(114);
    public static double H_150 = Math.toRadians(150);
    public static double H_180 = Math.toRadians(180);

    public Paths(Follower follower) {
        f = follower;
    }

    // -----------------------------
    // MIRROR
    // -----------------------------

    private double mirrorAngle(double a) {
        return Math.PI - a;
    }

    public void mirrorAll() {
        if (!IS_RED) return;

        P_START = P_START.mirror();
        P_PRELOAD_END = P_PRELOAD_END.mirror();
        P_AUDIENCE_SHOT = P_AUDIENCE_SHOT.mirror();

        P_I3_CP = P_I3_CP.mirror();
        P_I3_END = P_I3_END.mirror();
        P_I3_WALL = P_I3_WALL.mirror();

        P_I6_CP = P_I6_CP.mirror();
        P_I6_END = P_I6_END.mirror();
        P_I6_WALL = P_I6_WALL.mirror();

        P_RAMP_CP = P_RAMP_CP.mirror();
        P_RAMP_END = P_RAMP_END.mirror();
        P_RAMP_WALL = P_RAMP_WALL.mirror();
        P_RAMP_RETURN = P_RAMP_RETURN.mirror();

        P_I9_CP = P_I9_CP.mirror();
        P_I9_END = P_I9_END.mirror();
        P_I9_WALL = P_I9_WALL.mirror();
        P_I9_RETURN = P_I9_RETURN.mirror();

        P_EX_CP1 = P_EX_CP1.mirror();
        P_EX_END1 = P_EX_END1.mirror();
        P_EX_CP2 = P_EX_CP2.mirror();
        P_EX_END2 = P_EX_END2.mirror();
        P_EX_CP3 = P_EX_CP3.mirror();
        P_EX_END3 = P_EX_END3.mirror();

        P_LEAVE_END = P_LEAVE_END.mirror();

        H_90  = mirrorAngle(H_90);
        H_114 = mirrorAngle(H_114);
        H_150 = mirrorAngle(H_150);
        H_180 = mirrorAngle(H_180);
    }


    public void buildAll() {

        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_PRELOAD_END))
                .setLinearHeadingInterpolation(H_90, H_114)
                .build();

        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_PRELOAD_END, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_90, H_180)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        intake6 = f.pathBuilder()
                .addPath(new BezierCurve(P_AUDIENCE_SHOT, P_I6_CP, P_I6_END))
                .setLinearHeadingInterpolation(H_90, H_180)
                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot6 = f.pathBuilder()
                .addPath(new BezierCurve(P_I6_END, new Pose(56.722, 53.346), P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        unloadRamp = f.pathBuilder()
                .addPath(new BezierCurve(P_AUDIENCE_SHOT, P_RAMP_CP, P_RAMP_END))
                .setLinearHeadingInterpolation(H_90, H_180)
                .addPath(new BezierLine(P_RAMP_END, P_RAMP_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_RAMP_WALL, P_RAMP_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake9 = f.pathBuilder()
                .addPath(new BezierCurve(P_RAMP_RETURN, P_I9_CP, P_I9_END))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I9_END, P_I9_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I9_WALL, P_I9_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot9 = f.pathBuilder()
                .addPath(new BezierCurve(P_I9_RETURN, new Pose(45.580, 43.048), P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        intakeExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_AUDIENCE_SHOT, P_EX_CP1, P_EX_END1))
                .setLinearHeadingInterpolation(H_90, H_150)
                .build();

        shootExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END1, P_EX_CP1, P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_150, H_114)
                .build();

        intakeExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_AUDIENCE_SHOT, P_EX_CP2, P_EX_END2))
                .setLinearHeadingInterpolation(H_90, H_150)
                .build();

        shootExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END2, P_EX_CP2, P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_150, H_114)
                .build();

        intakeExtra3 = f.pathBuilder()
                .addPath(new BezierCurve(P_AUDIENCE_SHOT, P_EX_CP3, P_EX_END3))
                .setLinearHeadingInterpolation(H_90, H_150)
                .build();

        shootExtra3 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END3, P_EX_CP3, P_AUDIENCE_SHOT))
                .setLinearHeadingInterpolation(H_150, H_114)
                .build();

        leave = f.pathBuilder()
                .addPath(new BezierLine(P_AUDIENCE_SHOT, P_LEAVE_END))
                .setTangentHeadingInterpolation()
                .build();
    }
}
