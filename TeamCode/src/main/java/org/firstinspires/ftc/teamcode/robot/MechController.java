package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private MechState currentState;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Hardware constants
    private static final double MAX_SERVO_ROTATION = 300.0; // degrees

    // Limit constants
    private double lastIndexer = 1;
    // Offset constants
    private static final double INDEXER_OFFSET = 150;

    // Constructor
    public MechController(RobotHardware RoboRoar) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
        this.currentState = MechState.IDLE;
    }

    // State machine handler
    public void handleMechState(MechState state) {
        switch (state) {
            case IDLE:
                setIndexer(0);
                currentState = MechState.IDLE;
                break;

            case SHOOTING:
                setIndexer(0);
                currentState = MechState.SHOOTING;
                break;

            case INTAKE:
                setIndexer(0);
                currentState = MechState.INTAKE;
                break;

            case PURPLE_SORTING:
                setIndexer(0);
                currentState = MechState.PURPLE_SORTING;
                break;

            case GREEN_SORTING:
                setIndexer(0);
                currentState = MechState.GREEN_SORTING;
                break;

            case APRIL_TAG:
                setIndexer(0);
                currentState = MechState.APRIL_TAG;
                break;

            case HUMAN_STATE:
                setIndexer(0);
                currentState = MechState.HUMAN_STATE;
                break;
        }
    }

    // State machine methods
    public void setIndexer(double targetDegrees) {
        if (lastIndexer != targetDegrees) {
            double pos = (targetDegrees + INDEXER_OFFSET) / MAX_SERVO_ROTATION;
            pos = Math.max(0, Math.min(1, pos));
            robot.indexer.setPosition(pos);
            lastIndexer = targetDegrees;
        }
    }

    // Status
    public double statusIndexer(){
        return (robot.indexer.getPosition() * MAX_SERVO_ROTATION) - INDEXER_OFFSET;
    }
    public boolean isBusy() {
        return robot.intakeMot.isBusy() || robot.shootingMot.isBusy();
    }

    // Vision setup
    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(robot.cameraPosition, robot.cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .addProcessor(aprilTag)
                .build();
    }

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    // Vision controller
    public int findTagID(AprilTagProcessor aprilTag) {
        if (aprilTag == null || aprilTag.getDetections().isEmpty()) return 0;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            switch (detection.id) {
                case 21: return 1;
                case 22: return 2;
                case 23: return 3;
            }
        }
        return 0;
    }
    // Telemetry output
    public void allTelemetry() {
        telemetry.addData("State: ", currentState + " | Busy: " + isBusy());

        if (robot.pinpoint != null) {
            telemetry.addData("Pinpoint: ",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
            robot.pinpoint.update();
        }

        telemetry.addData("Indexer: ", statusIndexer());

        if (aprilTag != null && !aprilTag.getDetections().isEmpty()) {
            AprilTagDetection tag = aprilTag.getDetections().get(0);
            telemetry.addData("Tag ", "ID: %d | X: %.1f | Y: %.1f | Heading: %.1f°",
                    tag.id, tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.yaw);
        }

        telemetry.update();
    }
}