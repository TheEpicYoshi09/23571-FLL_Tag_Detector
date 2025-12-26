package org.firstinspires.ftc.teamcode.Opmodes.StateMachines;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriverManager {

    public enum DriveState {
        IDLE,       // nothing currently running
        MOVING,     // driving toward target position
        DONE        // cycle finished
    }

    public enum DriveResult {
        NONE,           // no cycle yet
        ARRIVED_OK,     // reached target pose successfully
        PATH_FAILED,    // couldn't reach target (timeout, large error)
        ABORTED         // cancelled by GameManager
    }

    // What kind of goal this cycle is about
    public enum DriveGoalKind {
        NONE,
        GOTO_BALL_SPOT,
        GOTO_SHOOT_SPOT,
        GOTO_PARK,
        PURE_ALIGN_TAG,   // e.g. only alignment around current spot
        DRIVE_FORWARD_FOR_INTAKE   // drive forward fixed distance while intaking
    }

    private final Follower follower;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private DriveState  state  = DriveState.IDLE;
    private DriveResult result = DriveResult.NONE;
    private DriveGoalKind goalKind = DriveGoalKind.NONE;

    // Target parameters for current cycle
    private Pose2D targetPose;
    private double timeoutSec = 5.0;

    // Current path being followed
    private Path currentPath;

    public DriverManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize Follower using PedroPathing Constants (leverages all configuration constants)
        this.follower = Constants.createFollower(hardwareMap);
    }

    // ---------- Public API for GameManager ----------

    public DriveState getState()        { return state; }
    public DriveResult getResult()      { return result; }
    public DriveGoalKind getGoalKind()  { return goalKind; }

    public boolean isIdle() { return state == DriveState.IDLE; }
    public boolean isDone() { return state == DriveState.DONE; }

    /** Hard stop + clear result. */
    public void resetCycle() {
        state     = DriveState.IDLE;
        result    = DriveResult.NONE;
        goalKind  = DriveGoalKind.NONE;
        currentPath = null;
    }

    public void abortCycle() {
        if (state == DriveState.IDLE) return;
        
        // Stop the follower immediately (set motors to zero)
        // This prevents the robot from continuing on the old path
        follower.setTeleOpDrive(0, 0, 0, true);
        
        state  = DriveState.DONE;
        result = DriveResult.ABORTED;
        currentPath = null;  // Clear current path reference
    }

    /**
     * Set the starting pose for path following.
     * Must be called before starting any paths (typically before waitForStart() in OpMode).
     */
    public void setStartingPose(Pose startingPose) {
        follower.setStartingPose(startingPose);
    }

    // ------------------------------------------------------------
    // START NEW CYCLES
    // ------------------------------------------------------------

    /** Generalized startCycle() for MOVE goals (ball spot, shoot spot, park). */
    public void startCycle(DriveGoalKind kind, Pose2D target, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = kind;
        this.targetPose = target;
        this.timeoutSec = timeoutSec;

        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        // Convert Pose2D to PedroPathing Pose
        Pose targetPedroPose = pose2DToPedroPose(target);
        
        // Get current robot pose
        Pose currentPose = follower.getPose();
        
        // Create path using BezierLine (straight line) from current to target
        BezierLine bezierLine = new BezierLine(currentPose, targetPedroPose);
        currentPath = new Path(bezierLine);
        
        // Set linear heading interpolation (like AutoPedroPath does)
        currentPath.setLinearHeadingInterpolation(
            currentPose.getHeading(),
            targetPedroPose.getHeading()
        );
        
        // Start following the path
        follower.followPath(currentPath);

        telemetry.addData("Drive", "Start MOVE cycle: %s to (%.1f, %.1f, %.1f°)",
                kind, target.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH),
                Math.toDegrees(target.getHeading(AngleUnit.RADIANS)));
    }

    /** startCycle() for ALIGN-ONLY mode (tag or heading). */
    public void startAlignCycle(double headingDeg, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.PURE_ALIGN_TAG;
        this.timeoutSec = timeoutSec;

        // For align-only, create a path that just adjusts heading at current position
        Pose currentPose = follower.getPose();
        double headingRad = Math.toRadians(headingDeg);
        Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), headingRad);
        
        BezierLine bezierLine = new BezierLine(currentPose, targetPose);
        currentPath = new Path(bezierLine);
        currentPath.setLinearHeadingInterpolation(currentPose.getHeading(), headingRad);
        
        follower.followPath(currentPath);

        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start ALIGN cycle to %.1f°", headingDeg);
    }

    /** Start forward drive for intake sequence - drives forward fixed distance from current position. */
    public void startForwardDrive(double distanceInch, double timeoutSec) {
        // Critical safety check: do not start new path if follower is still busy with previous path
        if (follower.isBusy()) {
            telemetry.addData("Drive/WARNING", "Cannot start forward drive - follower is still busy");
            return;
        }

        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.DRIVE_FORWARD_FOR_INTAKE;
        this.timeoutSec = timeoutSec;

        // Get current robot pose
        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading();

        // Calculate target pose: drive forward by distanceInch in the direction of current heading
        double newX = currentPose.getX() + distanceInch * Math.cos(heading);
        double newY = currentPose.getY() + distanceInch * Math.sin(heading);
        Pose targetPose = new Pose(newX, newY, heading); // maintain same heading

        // Create path using BezierLine (straight line) from current to target
        BezierLine bezierLine = new BezierLine(currentPose, targetPose);
        currentPath = new Path(bezierLine);

        // Set linear heading interpolation (maintain heading)
        currentPath.setLinearHeadingInterpolation(heading, heading);

        // Start following the path
        follower.followPath(currentPath);

        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start FORWARD_INTAKE drive: %.1f inches forward from (%.1f, %.1f)",
                distanceInch, currentPose.getX(), currentPose.getY());
    }

    // ------------------------------------------------------------
    // UPDATE LOOP (non-blocking)
    // ------------------------------------------------------------

    public void update(double nowSec) {
        // Update follower each cycle (required for path following to work)
        follower.update();

        if (state == DriveState.IDLE || state == DriveState.DONE)
            return;

        if (timer.seconds() > timeoutSec) {
            finish(DriveResult.PATH_FAILED);
            return;
        }

        switch (state) {
            case MOVING:
                // Check if path is complete using follower status
                if (!follower.isBusy()) {
                    // Path completed successfully
                    finish(DriveResult.ARRIVED_OK);
                }
                break;

            default:
                break;
        }
    }

    // ------------------------------------------------------------
    // FINISH HANDLER
    // ------------------------------------------------------------

    private void finish(DriveResult finalResult) {
        state = DriveState.DONE;
        result = finalResult;
        currentPath = null;

        telemetry.addData("Drive", "Cycle DONE: %s", finalResult);
    }

    // ------------------------------------------------------------
    // POSE CONVERSION HELPERS
    // ------------------------------------------------------------

    /**
     * Convert FTC Pose2D to PedroPathing Pose.
     */
    private Pose pose2DToPedroPose(Pose2D pose2D) {
        return new Pose(
            pose2D.getX(DistanceUnit.INCH),
            pose2D.getY(DistanceUnit.INCH),
            pose2D.getHeading(AngleUnit.RADIANS)
        );
    }
}
