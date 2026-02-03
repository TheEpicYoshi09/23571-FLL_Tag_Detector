package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;


/**
 * AprilTagVisionProcessor handles AprilTag detection to identify the target pattern.
 * It processes camera feed to detect AprilTags and interpret the sequence of colors.
 */
public class AprilTagVisionProcessor {

    // AprilTag IDs for different color patterns
    // These would need to be calibrated to your specific AprilTag setup
    private static final int APRIL_TAG_PURPLE_GREEN_GREEN = 1;
    private static final int APRIL_TAG_GREEN_PURPLE_GREEN = 2;
    private static final int APRIL_TAG_PURPLE_PURPLE_GREEN = 3;
    private static final int APRIL_TAG_GREEN_GREEN_PURPLE = 4;

    // Camera and vision portal references
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Detected pattern
    private String[] targetPattern = new String[3]; // Array to hold the 3-color pattern
    private boolean patternDetected = false;

    /**
     * Constructor for AprilTagVisionProcessor
     * Creates and configures the AprilTag processor and vision portal
     */
    public AprilTagVisionProcessor() {
        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

    }

    /**
     * Initializes the vision portal with the AprilTag processor
     * @param webcam The webcam hardware device
     */
    public void initVisionPortal(WebcamName webcam) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    /**
     * Gets the detected target pattern
     * @return Array of 3 strings representing the color pattern, or null if not detected
     */
    public String[] getTargetPattern() {
        if (patternDetected) {
            return targetPattern.clone(); // Return a copy to prevent external modification
        }
        return null;
    }

    /**
     * Checks if a pattern has been detected
     * @return true if pattern has been detected, false otherwise
     */
    public boolean isPatternDetected() {
        return patternDetected;
    }

    /**
     * Processes the camera feed to detect AprilTags and interpret the pattern
     * Call this method periodically to update detection status
     */
    public void processDetections() {
        if (visionPortal != null && aprilTagProcessor != null) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // Interpret the detected AprilTag to determine the pattern
                    interpretAprilTag(detection.id);

                    // Once we detect a pattern, we can stop looking for more tags for this cycle
                    break;
                }
            }
        }
    }

    /**
     * Interprets an AprilTag ID to determine the color pattern
     * @param tagId The AprilTag ID detected
     */
    private void interpretAprilTag(int tagId) {
        if (patternDetected) {
            // Pattern already detected, no need to process again
            return;
        }

        switch (tagId) {
            case APRIL_TAG_PURPLE_GREEN_GREEN:
                targetPattern[0] = "PURPLE";
                targetPattern[1] = "GREEN";
                targetPattern[2] = "GREEN";
                patternDetected = true;
                break;
            case APRIL_TAG_GREEN_PURPLE_GREEN:
                targetPattern[0] = "GREEN";
                targetPattern[1] = "PURPLE";
                targetPattern[2] = "GREEN";
                patternDetected = true;
                break;
            case APRIL_TAG_PURPLE_PURPLE_GREEN:
                targetPattern[0] = "PURPLE";
                targetPattern[1] = "PURPLE";
                targetPattern[2] = "GREEN";
                patternDetected = true;
                break;
            case APRIL_TAG_GREEN_GREEN_PURPLE:
                targetPattern[0] = "GREEN";
                targetPattern[1] = "GREEN";
                targetPattern[2] = "PURPLE";
                patternDetected = true;
                break;
            default:
                // Unknown tag, don't set pattern
                break;
        }
    }

    /**
     * Stops the vision portal
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Pauses the vision portal
     */
    public void pause() {
        if (visionPortal != null && aprilTagProcessor != null) {
            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        }
    }

    /**
     * Resumes the vision portal
     */
    public void resume() {
        if (visionPortal != null && aprilTagProcessor != null) {
            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        }
    }

    /**
     * Gets the underlying vision portal (for advanced operations if needed)
     * @return The VisionPortal instance
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}