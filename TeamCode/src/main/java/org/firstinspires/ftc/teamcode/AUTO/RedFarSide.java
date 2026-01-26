package org.firstinspires.ftc.teamcode.AUTO;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AUTO.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.AUTO.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "DECODEingAuto", group = "Autonomous")
public class RedFarSide extends LinearOpMode {
    private Limelight3A limelight;
    private static final int GPP = 21;
    private static final int PGP = 22;
    private static final int PPG = 23;

    // Store the last tag we saw

    public static double TARGET_RPM = 4900;
    LLResultTypes.FiducialResult tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d initialPose = new Pose2d(61, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5); // Set to your AprilTag pipeline
        limelight.start();
        telemetry.addLine("Waiting for start... scanning for tags");
        telemetry.update();

        Shooter.TARGET_RPM = 4900;

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
                                // ===== FIRST PART =====
                                new ParallelAction(
                                        drive.actionBuilder(initialPose)
                                                .lineToX(-38)
                                                .turn(Math.toRadians(-45))
                                                .build(),
                                        new ShooterActions.SpinUpUntilReady(shooter),
                                        new ShooterActions.HoldSpinForTime(shooter, 2),
                                        new ShooterActions.FireOnceTimed(shooter),
                                new ShooterActions.Stop(shooter)
                                ),
                                new ShooterActions.HoldSpinForTime(shooter, 2),
                                new ShooterActions.FireOnceTimed(shooter),

                                new IntakeActions.pushUp(intake),
                                new ShooterActions.HoldSpinForTime(shooter, 2),
                                new ShooterActions.FireOnceTimed(shooter),
                                new ShooterActions.Stop(shooter),

                                // ===== SECOND PART =====
                                new ParallelAction(
                                        drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                                .turn(Math.toRadians(45))
                                                .strafeToConstantHeading(new Vector2d(-30, 15))
                                                .turn(Math.toRadians(-90))
                                                .lineToY(40)
                                                .build(),
                                        new IntakeActions.takeIn(intake)
                                ),
                                new IntakeActions.stop(intake),

                                new ParallelAction(
                                    drive.actionBuilder(new Pose2d(-30, 40, Math.toRadians(-90)))
                                            .setTangent(Math.toRadians(-45))
                                        .splineTo(new Vector2d(-38, 15), Math.PI / 2)
                                            .build(),
                                        new ShooterActions.SpinUpUntilReady(shooter),
                                        new ShooterActions.HoldSpinForTime(shooter, 2),
                                        new ShooterActions.FireOnceTimed(shooter),
                                        new ShooterActions.Stop(shooter)
                                ),
                                new ShooterActions.HoldSpinForTime(shooter, 2),
                                new ShooterActions.FireOnceTimed(shooter),
                                new IntakeActions.pushUp(intake),
                                new ShooterActions.HoldSpinForTime(shooter, 2),
                                new ShooterActions.FireOnceTimed(shooter),
                                new ShooterActions.Stop(shooter),

                            new ParallelAction(
                        drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                .strafeToLinearHeading(new Vector2d(5,15), Math.toRadians(-150))
                                .build())
                        )
                );


            } else if (id == PGP) {
                      telemetry.addLine("Running PGP");
                      telemetry.update();

                 Actions.runBlocking(
                      new SequentialAction(
                              // ===== FIRST PART =====
                              new ParallelAction(
                                      drive.actionBuilder(initialPose)
                                              .lineToX(-38)
                                              .turn(Math.toRadians(-45))
                                              .build(),
                                      new ShooterActions.SpinUpUntilReady(shooter),
                                      new ShooterActions.HoldSpinForTime(shooter, 2),
                                      new ShooterActions.FireOnceTimed(shooter),
                                      new ShooterActions.Stop(shooter)
                              ),
                              new ShooterActions.HoldSpinForTime(shooter, 2),
                              new ShooterActions.FireOnceTimed(shooter),

                              new IntakeActions.pushUp(intake),
                              new ShooterActions.HoldSpinForTime(shooter, 2),
                              new ShooterActions.FireOnceTimed(shooter),
                              new ShooterActions.Stop(shooter),

                              // ===== SECOND PART =====
                              new ParallelAction(
                                      drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                              .turn(Math.toRadians(45))
                                              .strafeToConstantHeading(new Vector2d(0, 15))
                                              .turn(Math.toRadians(-90))
                                              .lineToY(40)
                                              .build(),
                                      new IntakeActions.takeIn(intake)
                              ),
                              new IntakeActions.stop(intake),

                              new ParallelAction(
                                      drive.actionBuilder(new Pose2d(0, 40, Math.toRadians(-90)))
                                              .setTangent(Math.toRadians(-45))
                                              .splineTo(new Vector2d(-38, 15), Math.PI / 2)
                                              .build(),
                                      new ShooterActions.SpinUpUntilReady(shooter),
                                      new ShooterActions.HoldSpinForTime(shooter, 2),
                                      new ShooterActions.FireOnceTimed(shooter),
                                      new ShooterActions.Stop(shooter)
                              ),
                              new ShooterActions.HoldSpinForTime(shooter, 2),
                              new ShooterActions.FireOnceTimed(shooter),
                              new IntakeActions.pushUp(intake),
                              new ShooterActions.HoldSpinForTime(shooter, 2),
                              new ShooterActions.FireOnceTimed(shooter),
                              new ShooterActions.Stop(shooter),

                              new ParallelAction(
                                      drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                              .strafeToLinearHeading(new Vector2d(5,15), Math.toRadians(-150))
                                              .build())
                      )
                 );

                        } else if (id == GPP) {
                            telemetry.addLine("Running GPP");
                            telemetry.update();

                            Actions.runBlocking(
                                    new SequentialAction(
                                            // ===== FIRST PART =====
                                            new ParallelAction(
                                                    drive.actionBuilder(initialPose)
                                                            .lineToX(-38)
                                                            .turn(Math.toRadians(-45))
                                                            .build(),
                                                    new ShooterActions.SpinUpUntilReady(shooter),
                                                    new ShooterActions.HoldSpinForTime(shooter, 2),
                                                    new ShooterActions.FireOnceTimed(shooter),
                                                    new ShooterActions.Stop(shooter)
                                            ),
                                            new ShooterActions.HoldSpinForTime(shooter, 2),
                                            new ShooterActions.FireOnceTimed(shooter),

                                            new IntakeActions.pushUp(intake),
                                            new ShooterActions.HoldSpinForTime(shooter, 2),
                                            new ShooterActions.FireOnceTimed(shooter),
                                            new ShooterActions.Stop(shooter),

                                            // ===== SECOND PART =====
                                            new ParallelAction(
                                                    drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                                            .turn(Math.toRadians(45))
                                                            .strafeToConstantHeading(new Vector2d(30, 15))
                                                            .turn(Math.toRadians(-90))
                                                            .lineToY(40)
                                                            .build(),
                                                    new IntakeActions.takeIn(intake)
                                            ),
                                            new IntakeActions.stop(intake),

                                            new ParallelAction(
                                                    drive.actionBuilder(new Pose2d(30, 40, Math.toRadians(-90)))
                                                            .setTangent(Math.toRadians(-45))
                                                            .splineTo(new Vector2d(-38, -15), Math.PI / 2)

                                                            .build(),
                                                    new ShooterActions.SpinUpUntilReady(shooter),
                                                    new ShooterActions.HoldSpinForTime(shooter, 2),
                                                    new ShooterActions.FireOnceTimed(shooter),
                                                    new ShooterActions.Stop(shooter)
                                            ),
                                            new ShooterActions.HoldSpinForTime(shooter, 2),
                                            new ShooterActions.FireOnceTimed(shooter),
                                            new IntakeActions.pushUp(intake),
                                            new ShooterActions.HoldSpinForTime(shooter, 2),
                                            new ShooterActions.FireOnceTimed(shooter),
                                            new ShooterActions.Stop(shooter),

                                            new ParallelAction(
                                                    drive.actionBuilder(new Pose2d(-38, 15, Math.toRadians(-45)))
                                                            .strafeToLinearHeading(new Vector2d(5,15), Math.toRadians(-150))
                                                            .build())
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