
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.Transform2d;
import org.firstinspires.ftc.teamcode.utils.Translation2d;

public class AutoDrive {
    public static final AutoDrive INSTANCE = new AutoDrive();
    private AutoDrive() { }

    HardwareMap hMap = null;
    Vision vision = null;
    Drive driveSys = null;
    Telemetry telemetry = null;
    ActiveOpMode opMode = null;
    Odometry odometry = null;

    private double          headingError  = 0;

    Pose2d cur_pose = null;
    Pose2d target_pose = null;
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    static final double     XY_THRESHOLD       = 1.0 ;
    static final double     HEADING_THRESHOLD  = 1.0 ;

    static final double     P_TURN_GAIN            = 0.020;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.025;     // Larger is more responsive, but also less stable

    public void init(Telemetry m_telemetry, HardwareMap hardwareMap) {

        opMode = ActiveOpMode.INSTANCE;
        telemetry = m_telemetry;
        hMap = hardwareMap;

        odometry = Odometry.INSTANCE;
        vision = Vision.INSTANCE;
        driveSys = Drive.INSTANCE;
        driveSys.init(hMap);
        driveSys.stop();

        cur_pose = odometry.getPose2d();
        target_pose = cur_pose;
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        cur_pose = odometry.getPose2d();
        double rot = cur_pose.getHeading();
        double x = Math.cos(distance * Math.sin(rot));
        double y = Math.sin(distance * Math.cos(rot));
        Translation2d trans_error = new Translation2d(x, y);
        Transform2d travel_pose = new Transform2d(trans_error,new Rotation2d(0.));
        target_pose = cur_pose.plus(travel_pose);
        double dist_error = trans_error.getNorm();
        if (opMode.isActive()) {
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveSys.moveRobot(maxDriveSpeed, 0, 0);
            while (opMode.isActive() && dist_error > XY_THRESHOLD) {
                cur_pose = odometry.getPose2d();
                trans_error = target_pose.minus(cur_pose).getTranslation();
                dist_error = trans_error.getNorm();
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0)
                    turnSpeed *= -1.0;
                driveSys.moveRobot(driveSpeed, 0, turnSpeed);
                sendTelemetry(true);
            }
            driveSys.stop();
        }
    }

    public void turnAndDrive(double maxTurnSpeed, double heading, double holdTime,
                             double maxDriveSpeed, double distance) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (opMode.isActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveSys.stop();
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opMode.isActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveSys.stop();
        this.driveStraight(maxDriveSpeed, distance, heading);
    }

    public void runSquareToTarget() {
        final double timeout = 3.;
        final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value

        int target_id = Vision.INSTANCE.target_id;
        ElapsedTime m_timer = new ElapsedTime();
        Vision.TargetPose targetPose = Vision.INSTANCE.targetPose;

        m_timer.reset();
        while (m_timer.seconds() < timeout) {
            targetPose = Vision.INSTANCE.getTargetPose();
            if (targetPose.id > 0) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (targetPose.pose.range - DESIRED_DISTANCE);
                double  headingError    = targetPose.pose.bearing;
                double  yawError        = targetPose.pose.yaw;
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                driveSys.moveRobot(drive, strafe, turn);
            }
        }
        driveStraight(MAX_AUTO_SPEED, 12., odometry.getHeading());
    }

    public void turnAndHoldHeading(double maxTurnSpeed, double heading, double holdTime) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (opMode.isActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveSys.stop();
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opMode.isActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveSys.stop();
        //odometry.update(0., heading);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (opMode.isActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveSys.stop();
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opMode.isActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveSys.moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }

        driveSys.stop();
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        headingError = targetHeading - odometry.getHeading();
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos X:Y",  "%7d:%7d", 0., 0.);
            telemetry.addData("Actual Pos X:Y",  "%7d:%7d", odometry.getX(), odometry.getY());
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                targetHeading, odometry.getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.update();
    }

    public void rampStraight(double maxDriveSpeed,
                             double minDriveSpeed,
                             double rampDist,
                             double distance,
                             double heading) {
        if (opMode.isActive()) {

            cur_pose = odometry.getPose2d();
            Pose2d start_pose = cur_pose;
            double rot = cur_pose.getHeading();
            double x = Math.cos(distance * Math.sin(rot));
            double y = Math.sin(distance * Math.cos(rot));
            Translation2d trans_error = new Translation2d(x, y);
            Transform2d travel_pose = new Transform2d(trans_error,new Rotation2d(0.));
            target_pose = cur_pose.plus(travel_pose);
            double dist_error = trans_error.getNorm();
            double fullDist = dist_error;

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            minDriveSpeed = Math.abs(minDriveSpeed);

            driveSys.moveRobot(minDriveSpeed, 0, 0);

            while (opMode.isActive() && dist_error > XY_THRESHOLD) {
                cur_pose = odometry.getPose2d();
                trans_error = target_pose.minus(cur_pose).getTranslation();
                dist_error = trans_error.getNorm();
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0)
                    turnSpeed *= -1.0;
                double dist_so_far = cur_pose.minus(start_pose).getTranslation().getNorm();
                driveSpeed = rampSpeed(minDriveSpeed, maxDriveSpeed, dist_so_far, rampDist, fullDist);
                driveSys.moveRobot(driveSpeed, 0, turnSpeed);
                sendTelemetry(true);
            }
            driveSys.stop();
        }
    }

    private double rampSpeed(double minSpeed, double maxSpeed,
                             double move, double ramp, double travel) {
       if (travel < ramp)  {
           return minSpeed +
                   ( travel / ramp )
                   * (maxSpeed - minSpeed);
       } else if ( (Math.abs(move) - travel) < ramp) {
           return minSpeed +
                   ( (Math.abs(move)-travel) / ramp )
                   * (maxSpeed - minSpeed);
       } else {
           return maxSpeed;
       }
    }
}
