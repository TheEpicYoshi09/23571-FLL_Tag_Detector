/*
*   Copyright (c) 2025 Alan Smith
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:

*   The above copyright notice and this permission notice shall be included in all
*   copies or substantial portions of the Software.

*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Rotation2d;

public class PinPoint {
    // Create an instance of the sensor
    public static final PinPoint INSTANCE = new PinPoint();

    private PinPoint() { }

    GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hMap) {
        // Get a reference to the sensor
        pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void setPose(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y,
                AngleUnit.DEGREES, heading));
    }

    public void setPose2d(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                                        AngleUnit.RADIANS, pose.getHeading()));
    }

    public void setPose2D(Pose2D pose) {
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
    }

    public Pose2d getPose2d() {
        pinpoint.update();
        Pose2D newpose = pinpoint.getPosition();
        return new Pose2d(newpose.getX(DistanceUnit.INCH),
                          newpose.getY(DistanceUnit.INCH),
                           new Rotation2d(newpose.getHeading(AngleUnit.RADIANS)));
    }

    public Pose2D getPose2D() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public double[] getPose() {
        double[] pose = {0.,0.,0.};
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        pose[0] = pose2D.getX(DistanceUnit.INCH);
        pose[1] = pose2D.getY(DistanceUnit.INCH);
        pose[2] = pose2D.getHeading(AngleUnit.DEGREES);
        return pose;
    }

    public void configurePinpoint(){
       /*
        *  Set the odometry pod positions relative to the point that you want the position to be measured from.
        *
        *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
        *  Left of the center is a positive number, right of center is a negative number.
        *
        *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
        *  Forward of center is a positive number, backwards is a negative number.
        */
        pinpoint.setOffsets(9, -4.25, DistanceUnit.CM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example: 
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
