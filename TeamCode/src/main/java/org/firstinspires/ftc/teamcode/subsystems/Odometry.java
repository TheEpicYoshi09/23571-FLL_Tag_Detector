
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Rotation2d;

public class Odometry {
    public static final Odometry INSTANCE = new Odometry();

    PinPoint pinpoint;
    private Odometry() { }

    private double robot_x;
    private double robot_y;
    private double robot_heading = 0;
    private boolean iAmBlue = true;

    public double getX() {
        return robot_x;
    }
    public double getY() {
        return robot_y;
    }
    public double getHeading() {
        return robot_heading;
    }

    public double getRot() {
        return fixRot(robot_heading * Math.PI / 180.);
    }

    public Pose2d getPose2d() {
        return new Pose2d(robot_x, robot_y, new Rotation2d(getRot()));
    }

    public void set(double x, double y, double heading) {
        robot_x = x;
        robot_y = y;
        robot_heading = heading;
        pinpoint.setPose(robot_x, robot_y, robot_heading);
    }

    public void set(Pose2d pose) {
        robot_x = pose.getX();
        robot_y = pose.getY();
        robot_heading = pose.getHeading() * 180. / Math.PI;
        pinpoint.setPose(robot_x, robot_y, robot_heading);
    }

    public void set(Pose2D pose) {
        pinpoint.setPose2D(pose);
        robot_x = pose.getX(DistanceUnit.INCH);
        robot_y = pose.getY(DistanceUnit.INCH);
        robot_heading = pose.getHeading(AngleUnit.DEGREES);
        pinpoint.setPose(robot_x, robot_y, robot_heading);
    }

    public void setHeading(double heading) {
        robot_heading = fixHeading(heading);
    }

    public void add(double x, double y) {
        robot_x += x;
        robot_y += y;
    }

    public void update() {
        Pose2d pose = pinpoint.getPose2d();
        robot_x = pose.getX();
        robot_y = pose.getY();
        robot_heading = radToDeg(fixRot(pose.getHeading()));
    }

    private double degToRad(double angle) {
        return angle * Math.PI / 180.;
    }
    private double radToDeg(double angle) {
        return angle / Math.PI * 180.;
    }

    private double fixHeading(double heading) {
        double fixedHeading = heading;
        while (fixedHeading <= -180.) {
            fixedHeading += 360.;
        }
        while (fixedHeading > 180.) {
            fixedHeading += 360.;
        }
        return fixedHeading;
    }

    private double fixRot(double rot) {
        double fixedRot = rot;
        while (fixedRot <= -Math.PI) {
            fixedRot += 2. * Math.PI;
        }
        while (fixedRot > Math.PI) {
            fixedRot += 2. * Math.PI;
        }
        return fixedRot;
    }

    public void init(boolean iAmBlue, boolean iAmAtGoal) {
        pinpoint = PinPoint.INSTANCE;
        if (iAmAtGoal) {
            if (iAmBlue) {
                robot_y = 0;
                robot_x = 0;
                robot_heading = 0;
            } else {
                robot_y = 0;
                robot_x = 0;
                robot_heading = 0;
            }
        } else {
            if (iAmBlue) {
                robot_y = 0;
                robot_x = 0;
                robot_heading = 0;
            } else {
                robot_y = 0;
                robot_x = 0;
                robot_heading = 0;
            }
        }
        set(robot_x, robot_y, robot_heading);
    }

    public void teleinit() {
        pinpoint = PinPoint.INSTANCE;
        update();
    }

}
