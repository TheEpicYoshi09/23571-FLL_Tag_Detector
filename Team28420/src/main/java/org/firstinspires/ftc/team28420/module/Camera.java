package org.firstinspires.ftc.team28420.module;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.types.AprilTag;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.util.Config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class Camera {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private MovementParams savedParams = new MovementParams(new PolarVector(0, 0), 0);

    public Camera(WebcamName webcamName) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTag);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    private List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    private PolarVector getVectorToPoint(double x, double y) {
        double y0 = y - Config.CameraConf.RANGE_TO_TAG;

        double theta = Math.atan2(y0, x);
        double abs = Math.hypot(x, y0) / 50;
        Config.Etc.telemetry.addData("abs", abs);
        return new PolarVector(theta, abs > 1 ? 1 : abs);
    }

    private PolarVector getVectorToPointWithFix(double x, double y, double bearing) {
        double y0 = y - Config.CameraConf.RANGE_TO_TAG;
        
        double cosB = Math.cos(-bearing);
        double sinB = Math.sin(-bearing);
        double xRotated = x * cosB - y0 * sinB;
        double yRotated = x * sinB + y0 * cosB;

        double theta = Math.atan2(xRotated, yRotated);
        double abs = Math.hypot(xRotated, yRotated) / 50;
        Config.Etc.telemetry.addData("abs", abs);
        return new PolarVector(theta, abs > 1 ? 1 : abs);
    }

    private double getRotateForce(double angle) {
        return angle / Config.CameraConf.ANGLE_MAX_VELOCITY;
    }

    public void updateApriltags() {
        for (AprilTagDetection detection : getDetections()) {
            if (AprilTag.fromId(detection.id) == AprilTag.RED) {
                savedParams = new MovementParams(getVectorToPointWithFix(detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(-detection.ftcPose.bearing)), getRotateForce(Math.toRadians(-detection.ftcPose.bearing)));
            }
            else {
                savedParams = new MovementParams(new PolarVector(0, 0), 0);
            }
        }
    }

    public MovementParams getSavedParams() {
        return savedParams;
    }

    public void log(Telemetry telemetry) {
        List<AprilTagDetection> detections = getDetections();

        telemetry.addLine("=== CAMERA ===");
        telemetry.addData("# apriltags detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.ROOT, "\n(ID: %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.ROOT, "X %6.1f, Y %6.1f, Z %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.ROOT, "P %6.1f, R %6.1f, Y %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.ROOT, "R %6.1f, B %6.1f, E %6.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.ROOT, "(ID: %d) UNKNOWN", detection.id));
            }
        }
        telemetry.addLine("=== CAMERA ===");
    }

    public void close() {
        visionPortal.close();
    }
}
