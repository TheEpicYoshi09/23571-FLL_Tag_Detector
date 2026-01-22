package org.firstinspires.ftc.teamcode.AUTO;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.AUTO.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.AUTO.ShooterActions;

import java.util.List;

@Config
@Autonomous(name = "DECODEingAuto", group = "Autonomous")
public class Blue1DECODETestAuto extends LinearOpMode {

    private DcMotorEx lfm, lbm, rfm, rbm;
    private DcMotorEx intake;
    private Shooter shooter;
    private CRServo bl, tl;
    private Limelight3A limelight;
    private static final int GPP = 21;
    private static final int PGP = 22;
    private static final int PPG = 23;

    // Store the last tag we saw
    LLResultTypes.FiducialResult tagOfInterest = null;

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
                } double pos = intake.getCurrentPosition();
                packet.put("IntakePos", pos);
                if (pos < 8000.0) {
                    return true;
                } else {
                    intake.setPower(0);
                    return false;
                }
            }
        } public Action takein() {
            return new TakeIn();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d initialPose = new Pose2d(61, -15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // topLauncher.moveUp(0.8, 2)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5); // Set to your AprilTag pipeline
        limelight.start();
        telemetry.addLine("Waiting for start... scanning for tags");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

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

                        if (id == PPG) {
                            telemetry.addLine("Running PPG");
                            telemetry.update();

                Actions.runBlocking(
                 new SequentialAction(
                 new com.acmerobotics.roadrunner.ParallelAction(
                    drive.actionBuilder(initialPose)
                         .lineToX(10)   // example
                         .build(),
                 new ShooterActions.SpinUpUntilReady(shooter)
                      ),
                 new ShooterActions.FireOnceTimed(shooter),
                 new ShooterActions.Stop(shooter)
                 ));


                        } else if (id == PGP) {
                            telemetry.addLine("Running PGP");
                            telemetry.update();

                            Actions.runBlocking(
                                    new SequentialAction(
                                            drive.actionBuilder(initialPose)
                                                    .lineToX(10)
                                                    .lineToX(-10)
                                                    .waitSeconds(2)
                                                    .build())
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
                                                    .build()// Shoot at the end
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