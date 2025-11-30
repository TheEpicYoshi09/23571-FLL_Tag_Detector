package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "Intake Loader Shooter Auto")
public class AutoIntakeLoaderShooter extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        robot.intake.intakeArtifacts(1.0);
        sleep(5000);
        robot.intake.stop();

        robot.loader.setLoaderMotor(1.0);
        sleep(5000);
        robot.loader.stop();

        robot.shooter.shootArtifacts();
        sleep(5000);
        robot.shooter.stop();
    }
}
