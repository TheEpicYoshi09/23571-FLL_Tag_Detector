package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSubsystem {
    private final Limelight3A limelight;

    // Raw Limelight values
    public boolean hasTarget = false;
    public boolean hasFieldPose = false;


    // Tag offsets
    private double tx;    // horizontal offset
    private double ty;    // vertical offset
    private double ta;    // target area

    // Distance to April Tag
    private double tagDistanceMeters = -1.0;
    private double tagForwardMeters = -1.0;   // absolute Z component if useful

    // fiducial count for telemetry/debugging
    private int fiducialCount = 0;


    // Field pose (meters, degrees)
    private double fieldX = 0.0;
    private double fieldY = 0.0;
    private double fieldYaw = 0.0;
    // Robot Yaw
    private double robotYawDeg = 0.0;

    // Lime Light Instance Builder
    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Poll up to 100x per second for fresh data
        limelight.setPollRateHz(100);

        // Start with pipeline 0 (configure in Limelight UI)
        limelight.pipelineSwitch(0);

        // Start background polling thread
        limelight.start();
    }

    // --- Change pipelines if you add more in the web UI. ---
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public double getSteeringCorrection() {
        if (!hasTarget) return 0.0;

        double kP = 0.03;  // start small and tune
        double correction = kP * tx;

        // Optional clamp so it doesn’t spin wildly
        double max = 0.3;
        if (correction > max) correction = max;
        if (correction < -max) correction = -max;

        return correction;
    }

    // --- Field Data ---
    public void updateRobotOrientation(double robotHeadingDeg) {
        robotYawDeg = robotHeadingDeg;

        double limelightYawDeg = -robotHeadingDeg;
        limelight.updateRobotOrientation(limelightYawDeg);
    }

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }

    public double getFieldYaw() {
        return fieldYaw;
    }

    public boolean hasFieldPose() {
        return hasFieldPose;
    }

    // --- Target Data Functions ---
    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTx() {
        return tx;
    }

    // Distance to April Tag
    public boolean hasTagDistance() {
        return tagDistanceMeters > 0;
    }

    public double getTagDistanceMeters() {
        return tagDistanceMeters;
    }


    public void update() {
        LLResult result = limelight.getLatestResult();

        // Early-out if no valid result
        if (result == null || !result.isValid()) {
            hasTarget = false;
            hasFieldPose = false;
            tagDistanceMeters = -1.0;
            tagForwardMeters = -1.0;
            fiducialCount = 0;
            return;
        }

        hasTarget = true;

        // 2D offsets
        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

        // Distance to April Tag (from fiducial results)
        if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            fiducialCount = result.getFiducialResults().size();

            LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
            Pose3D camToTag = tag.getCameraPoseTargetSpace();

            if (camToTag != null) {
                double x = camToTag.getPosition().x;
                double y = camToTag.getPosition().y;
                double z = camToTag.getPosition().z;

                // True straight-line distance (meters)
                tagDistanceMeters = Math.sqrt(x * x + y * y + z * z);

                // Absolute forward component (useful if z is forward/back)
                tagForwardMeters = Math.abs(z);
            } else {
                tagDistanceMeters = -1.0;
                tagForwardMeters = -1.0;
            }
        } else {
            fiducialCount = 0;
            tagDistanceMeters = -1.0;
            tagForwardMeters = -1.0;
        }

        // ---- 3D robot pose from AprilTags ----
        Pose3D botposeMT2 = result.getBotpose_MT2();
        if (botposeMT2 != null) {
            hasFieldPose = true;
            fieldX = botposeMT2.getPosition().x;
            fieldY = botposeMT2.getPosition().y;
            fieldYaw = botposeMT2.getOrientation().getYaw(AngleUnit.DEGREES);
        } else {
            // Fall back to basic botpose if MT2 is unavailable
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                hasFieldPose = true;
                fieldX = botpose.getPosition().x;
                fieldY = botpose.getPosition().y;
                fieldYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
            } else {
                hasFieldPose = false;
            }
        }
    }


    private static final double METERS_TO_FEET = 3.28084;

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Vision -----");
        telemetry.addData("Robot Yaw (deg)", robotYawDeg);
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.addLine("");
        telemetry.addData("Field X (m)", fieldX);
        telemetry.addData("Field Y (m)", fieldY);
        telemetry.addData("Has Distance", hasTagDistance());

        telemetry.addData("Fiducial Count", fiducialCount);

        if (tagDistanceMeters > 0) {
            telemetry.addData("Tag Dist (m) Euclidean", "%.3f", tagDistanceMeters);
            telemetry.addData("Tag Dist (ft) Euclidean", "%.3f",
                    tagDistanceMeters * METERS_TO_FEET);
        } else {
            telemetry.addData("Tag Dist (m) Euclidean", "n/a");
            telemetry.addData("Tag Dist (ft) Euclidean", "n/a");
        }

        if (tagForwardMeters > 0) {
            telemetry.addData("Tag Dist (m) ForwardZ", "%.3f", tagForwardMeters);
            telemetry.addData("Tag Dist (ft) ForwardZ", "%.3f",
                    tagForwardMeters * METERS_TO_FEET);
        } else {
            telemetry.addData("Tag Dist (m) ForwardZ", "n/a");
            telemetry.addData("Tag Dist (ft) ForwardZ", "n/a");
        }



    }
}

