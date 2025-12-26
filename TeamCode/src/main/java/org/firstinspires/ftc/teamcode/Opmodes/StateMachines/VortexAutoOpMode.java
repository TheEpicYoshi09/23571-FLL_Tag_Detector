package org.firstinspires.ftc.teamcode.Opmodes.StateMachines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Flipper;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;

@Autonomous(name = " State Machines Auto 0.01" +
        "", group = "Autonomous")
public class VortexAutoOpMode extends LinearOpMode {

    // Pose constants from AutoPedroPath
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(24, 24, Math.toRadians(0)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Shooting distances (in inches) - one per shoot position
    // TODO: Measure actual distances for each shoot position and update these values
    double INITIAL_SHOOT_DISTANCE = 45.0;  // Distance for initial/preload shoot
    double[] SHOOT_DISTANCES = {45.0, 45.0, 45.0};  // Distances for each cycle shoot (matches SHOOT_POS array)
    double DEFAULT_SHOOT_DISTANCE = 45.0;  // Ultimate fallback distance
        
    // Intake forward distance (in inches) - fixed distance to drive forward while intaking at each ball spot
    // This represents the distance needed to collect 3 balls in a row before hitting the field wall
    // TODO: Measure actual field distance and update this value
    double INTAKE_FORWARD_DISTANCE = 36.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- create hardware-level objects ---
        Intake intake = new Intake();
        FlyWheel flyWheel = new FlyWheel();
        Flipper flipper = new Flipper();
        Kicker kicker = new Kicker();
        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

        // --- build high-level managers ---
        // Use dryRun=true to skip hardware calls when hardware is not initialized
        IntakeManager intakeManager = new IntakeManager(intake, telemetry, true);
        ShootManager shootManager = new ShootManager(flyWheel, kicker, flipper, intake, telemetry, true);
        
        // DriverManager now uses HardwareMap (Follower initialized internally using PedroPathing Constants)
        DriverManager driveManager = new DriverManager(hardwareMap, telemetry);

        // Reset odometry position and IMU before setting starting pose
        // This ensures accurate localization at the start of autonomous
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        // Set starting pose BEFORE waitForStart() (matches AutoPedroPath's init() pattern)
        driveManager.setStartingPose(startPose);

        // Convert poses for GameManager arrays
        // Initial setup matching AutoPedroPath's scorePreload sequence
        Pose2D[] BALL_POS = {pedroPoseToPose2D(pickup1Pose), pedroPoseToPose2D(pickup2Pose), pedroPoseToPose2D(pickup3Pose)};  
        Pose2D[] SHOOT_POS = {pedroPoseToPose2D(scorePose),pedroPoseToPose2D(scorePose),pedroPoseToPose2D(scorePose)};  // three shoot spots 
        Pose2D PARK_POS = pedroPoseToPose2D(scorePose);  // Park at score pose
        Pose2D INITIAL_SHOOT_POS = pedroPoseToPose2D(scorePose);  // Initial/preload shoot position

        // Initialize GameManager
        GameManager gameManager = new GameManager(
            driveManager,
            intakeManager,
            shootManager,
            telemetry,
            BALL_POS,
            SHOOT_POS,
            PARK_POS,
            INITIAL_SHOOT_POS,
            30,  // autoTotalTimeSec
            5,   // parkReserveSec
            3,   // ballsPerSpot
            INITIAL_SHOOT_DISTANCE,
            SHOOT_DISTANCES,
            DEFAULT_SHOOT_DISTANCE,
            INTAKE_FORWARD_DISTANCE,  // intakeForwardDistanceInch
            4,   // driveTimeoutSec
            3,   // intakeTimeoutSec
            3    // shootTimeoutSec
        );

        // Wait for start (LinearOpMode requirement)
        waitForStart();

        // Begin autonomous sequence after start (CRITICAL: Must be called for time-based logic)
        gameManager.startAuto(getRuntime());

        // Main update loop
        // NOTE: Update driveManager first so gameManager can react immediately to drive completion
        while (opModeIsActive() && !gameManager.isDone()) {
            double nowSec = getRuntime();

            driveManager.update(nowSec);     // local FSM: drive (calls follower.update() internally) - UPDATE FIRST
            intakeManager.update(nowSec);    // local FSM: intake
            shootManager.update(nowSec);     // local FSM: shooter
            gameManager.update(nowSec);      // global phase FSM - reacts to drive completion

            telemetry.update();
        }
    }

    /**
     * Convert PedroPathing Pose to FTC Pose2D.
     * CRITICAL: Pose2D constructor requires explicit DistanceUnit and AngleUnit parameters.
     */
    private Pose2D pedroPoseToPose2D(Pose pedroPose) {
        // Pose2D constructor: new Pose2D(DistanceUnit, double x, double y, AngleUnit, double heading)
        return new Pose2D(
            DistanceUnit.INCH,        // Explicit unit - PedroPathing Pose uses inches
            pedroPose.getX(),         // X coordinate in inches
            pedroPose.getY(),         // Y coordinate in inches
            AngleUnit.RADIANS,        // Explicit unit - PedroPathing Pose uses radians
            pedroPose.getHeading()    // Heading in radians
        );
    }
}
