package org.firstinspires.ftc.teamcode.AUTO;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "BADauto", group = "Autonomous")
public class BlueAuto1 extends LinearOpMode {

    private DcMotorEx lfm, lbm, rfm, rbm;
    private DcMotorEx intake, lift, sl, sr;
    private CRServo bl, tl;
    private Limelight3A limelight;
    private static final int GPP = 21;
    private static final int PGP = 22;
    private static final int PPG = 23;

    // Store the last tag we saw
    LLResultTypes.FiducialResult tagOfInterest = null;

    // ==============================
    // Mechanism: Shooter
    // ==============================
    public class Shooter {
        public Shooter(HardwareMap hardwareMap) {
            sr = hardwareMap.get(DcMotorEx.class, "sr");
            sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sr.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class Shoot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    sr.setPower(0.8);
                    initialized = true;
                }

                double pos = sr.getCurrentPosition();
                packet.put("ShooterPos", pos);

                if (pos < 3000.0) {
                    return true;
                } else {
                    sr.setPower(0);
                    return false;
                }
            }
        }

        public Action shoot() {
            return new Shoot();
        }
    }

    // ==============================
    // Mechanism: Intake
    // ==============================
    public class Intake {
        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class TakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.8);
                    initialized = true;
                }

                double pos = intake.getCurrentPosition();
                packet.put("IntakePos", pos);

                if (time < 3) {
                    return true;
                } else {
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action takein() {
            return new TakeIn();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5); // Set to your AprilTag pipeline
        limelight.start();

        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        telemetry.addLine("Waiting for start... scanning for tags");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        // ========================================
        // 1. Detect AprilTag before starting motion
        // ========================================
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f == null) continue;
                    int id = f.getFiducialId();

                    if (id == PPG || id == PGP || id == GPP) {
                        tagOfInterest = f;
                        telemetry.addData("AprilTag Found", id);
                        telemetry.update();

                        // ============================
                        // 2. Run specific trajectories
                        // ============================
                        if (id == PPG) {
                            telemetry.addLine("Running PPG");
                            telemetry.update();

                            Actions.runBlocking(
                                    /*.lineToX(50)
                           .turn(Math.toRadians(180))
                           .lineToX(-20)
                           .turn(Math.toRadians(50))
                           //shoot balls
                           .waitSeconds(1.5)                                                    .lineToX(-10)
                           .turn(Math.toRadians(40))
                           .lineToY(-40)
                           .waitSeconds(0.2) //intake balls                                                  .lineToY(-50)
                           .setTangent(40)
                           .splineTo(new Vector2d(-20, -15), Math.PI / 2)
                           //shoot balls
                           .waitSeconds(1.5)
                           .setTangent(0)
                           .splineToConstantHeading(new Vector2d(5, 20), Math.PI / 2)
                           .build()*/
                                    new SequentialAction(
                                            drive.actionBuilder(initialPose)
                                                    .lineToX(50)
                                                    .turn(Math.toRadians(180))
                                                    .lineToX(-20)
                                                    .turn(Math.toRadians(50))
                                                    .build(),
                                            shooter.shoot(), // Shoot sequence
                                            drive.actionBuilder(new Pose2d(-20, 0, Math.toRadians(50)))
                                                    .turn(Math.toRadians(40))
                                                    .lineToY(-40)
                                                    .build(),
                                            intake.takein(),
                                            drive.actionBuilder(new Pose2d(-20, 0, Math.toRadians(50)))
                                                    .lineToY(-50)
                                                    .setTangent(40)
                                                    .build(),
                                            shooter.shoot(),
                                            drive.actionBuilder(new Pose2d(-20, -15, Math.toRadians(40)))
                                                    .setTangent(0)
                                                    .splineToConstantHeading(new Vector2d(5, 20), Math.PI / 2)
                                                    .build()
                                            // Shoot again
                                    )
                            );

                        } else if (id == PGP) {
                            telemetry.addLine("Running PGP");
                            telemetry.update();

                            Actions.runBlocking(
                                    new SequentialAction(
                                            drive.actionBuilder(initialPose)
                                                    .lineToX(10)
                                                    .lineToX(-10)
                                                    .waitSeconds(2)
                                                    .build(),
                                            shooter.shoot()
                                    )
                            );

                        } else if (id == GPP) {
                            telemetry.addLine("Running GPP");
                            telemetry.update();

                            Actions.runBlocking(
                                    new SequentialAction(
                                            drive.actionBuilder(initialPose)
                                                    .setTangent(0)
                                                    .splineToSplineHeading(new Pose2d(30, 30, 0), Math.PI / 2)
                                                    .waitSeconds(2)
                                                    .build(),

                                            intake.takein(), // Intake first

                                            drive.actionBuilder(new Pose2d(30, 30, 0))
                                                    .lineToX(60)
                                                    .build(),

                                            shooter.shoot() // Shoot at the end
                                    )
                            );
                        }
                    }
                }
            }
        } else {
            telemetry.addLine("No AprilTag detected at start.");
            telemetry.update();
        }
    }
}


/*package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;


@Config
@Autonomous(name = "DECODETestAuto", group = "Autonomous")
public class Blue1DECODETestAuto extends LinearOpMode {

    private DcMotorEx lfm, lbm, rfm, rbm, intake, shooter;
    private Limelight3A limelight;
    private static final int GPP = 21;
    private static final int PGP = 22;
    private static final int PPG = 23;
    // private static final int blue = 20;
    //private static final int red  = 24;

    // Store the last tag we saw
    // Store the last tag we saw
    LLResultTypes.FiducialResult tagOfInterest = null;

    public class Shooter {
        public Shooter(HardwareMap hardwareMap) {
            shooter = hardwareMap.get(DcMotorEx.class, "liftMotor");
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class shoot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooter.setPower(0.8);
                    initialized = true;
                }
                double pos = shooter.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    shooter.setPower(0);
                    return false;
                }
            }
        }
        public Action shoot() {
            return new shoot();
        }
    }

    public class Intake {
        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "liftMotor");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class takein implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.8);
                    initialized = true;
                }
                double pos = intake.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    intake.setPower(0);
                    return false;
                }
            }
        }
        public Action takein() {
            return new takein();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        // make a Lift instance
        Intake intake = new Intake(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5); // Set to your AprilTag pipeline


        limelight.start();
        // Init Road Runner drive
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Start pose (adjust for your field setup)


        telemetry.addLine("Waiting for start... scanning for tags");
        telemetry.update();

        // Wait for play
        waitForStart();

        if (isStopRequested()) return;

        // ============================
        // 1. Detect AprilTag
        // ============================
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("No result from Limelight");
            } else {
                telemetry.addData("Result valid?", result.isValid());
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials == null || fiducials.isEmpty()) {
                    telemetry.addLine("No fiducials detected");
                } else {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        telemetry.addData("Fiducial ID", f.getFiducialId());
                    }
                }
            }
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    boolean tagSeen = false;
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f == null) continue;
                        int id = f.getFiducialId();

                        if (id == PPG || id == PGP || id == GPP) {
                            tagOfInterest = f;
                            tagSeen = true;

                            telemetry.addData("AprilTag Found", id);
                            // Init Road Runner drive

                            // Start pose (adjust for your field setup)

                            waitForStart();

                            if (isStopRequested()) return;

                            // ============================
                            // 1. Detect AprilTag
                            // ============================
                            while (opModeIsActive()) {
                                // Example: run paths when tags appear
                                if (id == PPG) {
                                    telemetry.addLine("Running PPG");
                                    Actions.runBlocking(new SequentialAction(
                                            drive.actionBuilder(initialPose)
                                                    .lineToX(50)
                                                    .turn(Math.toRadians(180))
                                                    .lineToX(-20)
                                                    .turn(Math.toRadians(50))
                                                    //shoot balls
                                                    .waitSeconds(1.5)
                                                    .lineToX(-10)
                                                    .turn(Math.toRadians(40))
                                                    .lineToY(-40)
                                                    .waitSeconds(0.2) //intake balls
                                                    .lineToY(-50)
                                                    .setTangent(40)
                                                    .splineTo(new Vector2d(-20, -15), Math.PI / 2)
                                                    //shoot balls
                                                    .waitSeconds(1.5)
                                                    .setTangent(0)
                                                    .splineToConstantHeading(new Vector2d(5, 20), Math.PI / 2)
                                                    .build()
                                    )
                                    );

                                } else if (id == PGP) {
                                    telemetry.addLine("Running PGP");
                                    Actions.runBlocking(
                                            new SequentialAction(
                                                    drive.actionBuilder(initialPose)
                                                            .lineToX(10)
                                                            .lineToX(-10)
                                                            .waitSeconds(2)
                                                            .build()
                                            )
                                    );
                                } else if (id == GPP) {
                                    telemetry.addLine("Running GPP");
                                    Actions.runBlocking(
                                            new SequentialAction(
                                                    drive.actionBuilder(initialPose)
                                                            .setTangent(0)
                                                            .splineToSplineHeading(new Pose2d(30, 30, 0), Math.PI / 2)
                                                            .waitSeconds(2)
                                                            .build()
                                            )
                                    );
                                    Actions.runBlocking(intake.takein());


                                    Actions.runBlocking(
                                            new SequentialAction(
                                                    trajectoryActionChosen,
                                                    Shooter.shoot(),
                                                    Intake.takein(),

                                                    trajectoryActionCloseOut
                                            ));
                                }
                            }
                        }
                        if (!tagSeen) {
                            telemetry.addLine("No AprilTag in view");
                        }
                    }

                    telemetry.update();
                }
            }


        }}}*/