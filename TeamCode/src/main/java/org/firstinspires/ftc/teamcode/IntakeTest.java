package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArtifactTracker;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        ArtifactTracker artifactTracker = new ArtifactTracker(robot, telemetry);
        IntakeController intakeController = new IntakeController(robot, artifactTracker, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            intakeController.update();

            if (gamepad1.right_bumper && !intakeController.isSpindexerFull()) {
                robot.runIntake(RobotHardware.IntakeDirection.IN);
            } else if (gamepad1.left_bumper) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            telemetry.update();
        }

        robot.runIntake(RobotHardware.IntakeDirection.STOP);
    }
}
