package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class Red {
    // Auto
    // Start Pose of our robot.
    public static final Pose START_POSE = new Pose(80.6, 9, Math.toRadians(90));

    // Scoring Pose of our robot. It is facing the goal at a 45 degree angle.
    public static final Pose SCORE_POSE = new Pose(86, 90, Math.toRadians(45));

    // Highest (First Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN1_POSE = new Pose(102.5, 84, Math.toRadians(180));
    public static final Pose PICKUP1_POSE = new Pose(117.5, 84, Math.toRadians(180));

    // Middle (Second Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN2_POSE = new Pose(102.5, 60, Math.toRadians(180));
    public static final Pose PICKUP2_POSE = new Pose(117.5, 60, Math.toRadians(180));

    // Lowest (Last Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN3_POSE = new Pose(102.5, 36, Math.toRadians(180));
    public static final Pose PICKUP3_POSE = new Pose(117.5, 36, Math.toRadians(180));

    // Teleop
    // Gate Start & End
    public static final Pose GATE_START_POSE = new Pose(126, 72, Math.toRadians(180));
    public static final Pose GATE_END_POSE = new Pose(129, 72, Math.toRadians(180));

    // Endgame
    public static final Pose ENDGAME_POSE = new Pose(38, 33, Math.toRadians(90));

    private Red() {} // Prevent instantiation
}
