package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class DecodePaths {


    ///  BLUE FAR
    public static final Pose BLUE_FAR_START = new Pose(60, 8.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SHOOT = new Pose(60, 18.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SHOOT_TO_SPIKE3 = new Pose(48.25, 34.5, Math.toRadians(180));
    public static final Pose BLUE_FAR_PICKUP_SPIKE3_PART1 = new Pose(34, 34.5, Math.toRadians(180));
    public static final Pose BLUE_FAR_PICKUP_SPIKE3_PART2 = new Pose(25.5, 34.5, Math.toRadians(180));
    public static final Pose BLUE_FAR_LEAVE = new Pose(60, 34.5, Math.toRadians(-90));


    /// RED FAR
    public static final Pose RED_FAR_START = new Pose(84, 8.75, Math.toRadians(-90));
    public static final Pose RED_FAR_SHOOT = new Pose(84, 18.75, Math.toRadians(-90));
    public static final Pose RED_FAR_SHOOT_TO_SPIKE3 = new Pose(96.75, 34.5, 0);
    public static final Pose RED_FAR_PICKUP_SPIKE3_PART1 = new Pose(111, 34.5, 0);
    public static final Pose RED_FAR_PICKUP_SPIKE3_PART2 = new Pose(118, 34.5, 0);
    public static final Pose RED_FAR_LEAVE = new Pose(84, 34.5, Math.toRadians(-90));


    /// BLUE CLOSE
    public static final Pose BLUE_NEAR_START = new Pose(23,119.8,Math.toRadians(-90));
    public static final Pose BLUE_NEAR_SHOOT = new Pose(45, 98, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_GOTO_ARTIFACTS = new Pose(24.1, 98, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_PICKUP_ARTIFACTS = new Pose(24.1, 90, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_GO_INSIDE_ZONE = new Pose(60, 132.5, Math.toRadians(-90));

    ///  RED CLOSE
    public static final Pose RED_NEAR_START = new Pose(121,119.8,Math.toRadians(-90));
    public static final Pose RED_NEAR_SHOOT = new Pose(98.5, 98, Math.toRadians(-90));
    public static final Pose RED_NEAR_GOTO_ARTIFACTS = new Pose(119.8, 98, Math.toRadians(-90));
    public static final Pose RED_NEAR_PICKUP_ARTIFACTS = new Pose(119.8, 90, Math.toRadians(-90));
    public static final Pose RED_NEAR_GO_INSIDE_ZONE = new Pose(84, 132.5, Math.toRadians(-90));

    /// BLUE CLOSE LEAVE ONLY
    public static final Pose BLUE_NEAR_FROM_SHOOT_TO_WALL = new Pose(59, 133.6, Math.toRadians(-90));

    // RED CLOSE LEAVE ONLY
    public static final Pose RED_NEAR_FROM_SHOOT_TO_WALL = new Pose(85, 133.6, Math.toRadians(-90));
}
