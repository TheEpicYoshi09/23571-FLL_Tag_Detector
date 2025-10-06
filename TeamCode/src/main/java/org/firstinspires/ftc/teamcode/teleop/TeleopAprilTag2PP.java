package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class TeleopAprilTag2PP extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Follower follower;
    private boolean following = false;

    private final Pose startPose = new Pose(56, 32, Math.toRadians(0));
    private final Pose TARGET_LOCATION = new Pose(58, 90, Math.toRadians(180));

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addData("Init Pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.resumeStreaming();
    }

    @Override
    public void loop() {
        follower.update();

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

        // 🔹 Update robot pose from AprilTag
        Pose updatedPose = getRobotPoseFromCamera();
        if (updatedPose != null) {
            follower.setPose(updatedPose);
        }

        // Stop following if done
        if (following && !follower.isBusy()) {
            following = false;
        }

        telemetry.addData("Pedro Pose", follower.getPose());
        telemetryAprilTag();
        telemetry.update();
    }

    /**
     * Convert FTC AprilTag (camera-relative) pose → Pedro field pose.
     */
    private Pose getRobotPoseFromCamera() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0); // use first or best detection
            /*
            tagX = 13.65, 16.37, 54
            Pose GoalBlue = 13.65, 127.63, 144
            Pose GoalRed = 130.35, 127.63, 36

             */

            // Only process Tag 20 (Blue side)
            if (tag.id == 20) {
                // --- Step 1: Tag 20 field pose (in inches, FTC-centered coordinates) ---
                double tagX = -58.35;   // X (toward red)
                double tagY = -55.63;   // Y (toward back wall)
                double tagHeadingDeg = 54.0;

                // --- Step 2: Camera-relative FTC pose (convert from meters → inches) ---
                double camX = tag.ftcPose.x * 39.3701; // forward
                double camY = tag.ftcPose.y * 39.3701; // left
                double camHeadingDeg = tag.ftcPose.yaw;

                // --- Step 3: Convert degrees → radians ---
                double tagHeading = Math.toRadians(tagHeadingDeg);
                double camHeading = Math.toRadians(camHeadingDeg);

                // --- Step 4: Compute robot pose on field (planar math) ---
                // Robot position = tag position - rotated camera vector
                double robotX = tagX - (camX * Math.cos(tagHeading) - camY * Math.sin(tagHeading));
                double robotY = tagY - (camX * Math.sin(tagHeading) + camY * Math.cos(tagHeading));
                double robotHeading = tagHeading - camHeading;

                // --- Step 5: Convert FTC field coords (center ±72) → Pedro coords (0..144) ---
                double pedroX = robotX + 72.0;
                double pedroY = robotY + 72.0;

                Pose pedroPose = new Pose(pedroX, pedroY, robotHeading);

                telemetry.addLine("Using Tag 20 for pose update");
                telemetry.addData("TagYaw(deg)", tag.ftcPose.yaw);
                telemetry.addData("FTC Pose (inch)", "(%.1f, %.1f)", robotX, robotY);
                telemetry.addData("Pedro Pose (inch)", "(%.1f, %.1f)", pedroX, pedroY);

                return pedroPose;
            }
        }
        return null; // no valid tag
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %.1f %.1f %.1f (m)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %.1f %.1f %.1f (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %.0f %.0f (px)", detection.center.x, detection.center.y));
            }
        }
    }
}

