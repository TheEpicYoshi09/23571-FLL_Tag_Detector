package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.subsystems.Subsystem;

public class MecDriveSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;
    public static boolean debug = false;
    private Follower follower;

    //For calculated turret angle
    private static double changeInTurretAngle = 0;


    public MecDriveSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, UniConstants.teamColor color){
        this.color = color;
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose());
        follower.update();

    }




    @Override
    public void periodic(){
        follower.update();
    }

    public void startTele(){
        follower.startTeleopDrive();
        follower.update();
    }
    public void updateTeleop(double forward, double strafe, double rotation, boolean botCentric){
        follower.setTeleOpDrive(forward, strafe, rotation, botCentric);
        follower.update();
    }

    public double updateDistanceAndAngle(UniConstants.teamColor color) {
        //Returns distance between goal in meters, it also updates the turretTargetAngle
        double x = 0,y = 1;
        switch (color){
            case BLUE:
                x = follower.getPose().getX() - Poses.blueGoal.getX();
                y = follower.getPose().getY() - Poses.blueGoal.getY();
                break;
            case RED:
                x = follower.getPose().getX() - Poses.redGoal.getX();
                y = follower.getPose().getY() - Poses.redGoal.getY();
        }

        changeInTurretAngle = (Math.toDegrees(Math.atan2(x,y))) + follower.getHeading();
        return Math.hypot(x,y) / 39.37;

    }

    public double getCalculatedTurretAngle(){
        return changeInTurretAngle;
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF MEC DRIVE LOG");
                telemetry.addData("Pose X ", follower.getPose().getX());
                telemetry.addData("Pose Y ", follower.getPose().getY());
                telemetry.addData("Pose Heading ", follower.getPose().getHeading());
                telemetry.addLine("END OF MEC DRIVE LOG");
                break;
            case EXTREME:
                telemetry.addLine("START OF MEC DRIVE LOG");
                telemetry.addData("Follower Debug ", follower.debug());
                telemetry.addLine("END OF MEC DRIVE LOG");
                break;
        }
    }

}
