package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Limelight USB AprilTag Distance", group = "Vision")
public class AprilTagDistance extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ⚙️ Camera/field constants
    private static final double CAMERA_HEIGHT_METERS = 0.28;  // Camera height
    private static final double TAG_HEIGHT_METERS = 0.78;     // Basket tag height
    private static final double CAMERA_ANGLE_DEGREES = 14.0;  // Camera tilt

    // Example basket tag IDs
    private static final int[] BASKET_TAG_IDS = {20, 24};
    private static final String LIMELIGHT_URL = "http://10.0.0.11:5801";
    private static final int LIMELIGHT_PIPELINE = 6;
    @Override
    public void runOpMode() {

        // ✅ Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        // ✅ Initialize VisionPortal with Limelight as USB camera


        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            AprilTagDetection tag = getBasketTag();

            if (tag != null) {
                double x = tag.ftcPose.x;  // Forward from camera
                double y = tag.ftcPose.y;  // Left/right
                double z = tag.ftcPose.z;  // Up/down

                double flatDistance = Math.hypot(x, y);

                telemetry.addLine("🎯 Basket AprilTag Detected");
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X (Forward m)", x);
                telemetry.addData("Y (Sideways m)", y);
                telemetry.addData("Z (Up m)", z);
                telemetry.addData("Flat Distance (m)", flatDistance);
            } else {
                telemetry.addLine("No Basket AprilTag Detected");
            }

            telemetry.update();
        }
    }

    /**
     * Returns the first basket tag detected.
     */
    private AprilTagDetection getBasketTag() {
        for (AprilTagDetection tag : aprilTag.getDetections()) {
            for (int id : BASKET_TAG_IDS) {
                if (tag.id == id) return tag;
            }
        }
        return null;
    }
}
