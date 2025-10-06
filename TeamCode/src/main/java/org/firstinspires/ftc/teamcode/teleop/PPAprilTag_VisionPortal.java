package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class PPAprilTag_VisionPortal extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;   // FTC SDK AprilTag processor
    private Follower follower;
    private boolean following = false;

    private final Pose startPose = new Pose(56, 32, Math.toRadians(0));
    private final Pose TARGET_LOCATION = new Pose(58, 90, Math.toRadians(180));

    @Override
    public void init() {
        // Build AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create VisionPortal using Logitech C270 webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // name from RC config
                .addProcessor(aprilTag)
                .build();

        // Pedro Pathing Follower setup
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("position", follower.getPose());
        telemetryAprilTag();
        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.resumeStreaming(); // Start the webcam stream
    }

    @Override
    public void loop() {
        follower.update();

        // Build path once and follow
        if (!following) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta()
                            )
                            .build()
            );
            following = true;
        }

        // Update robot pose from AprilTag camera data
        Pose updatedPose = getRobotPoseFromCamera();
        if (updatedPose != null) {
            follower.setPose(updatedPose);
        }

        // Stop following if done
        if (following && !follower.isBusy()) {
            following = false;
        }
        telemetry.addData("position", follower.getPose());
        telemetryAprilTag();
        telemetry.update();
    }

    private Pose getRobotPoseFromCamera() {
        // Grab AprilTag detections
        List<AprilTagDetection> detections = aprilTag.getDetections();
        /*
        Tag            	X   	            Y	                Z	            Yaw
        20 (Blue)	-1.482m (-58.35in)	-1.413m (-55.63in)	0.749m (29.49in)	 54°
        24 (Red)	-1.482m (-58.35in)	1.413m	(55.63in)   0.749m (29.49in)    -54°
         */

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0); // use first detection (or add logic for best one)

            // The FTC AprilTag API gives you pose in meters relative to camera
            double x = tag.ftcPose.x;
            double y = tag.ftcPose.y;
            double heading = Math.toRadians(tag.ftcPose.yaw);

            // Convert FTC coordinate system → Pedro Pathing coordinate system
            return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }

        return null; // no detection
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}
