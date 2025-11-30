package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "6 Balls Auto Blue")
public class SixBallsAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // ----------- FIRST 3 SHOTS (Shooter revs FIRST) -----------
        robot.shooter.shootArtifacts();   // rev up shooter
        sleep(1200);                      // build RPM before feeding

        for (int i = 0; i < 3; i++) {
            robot.loader.setLoaderMotor(1.0);   // feed ball
            sleep(900);                          // feeding time

            robot.loader.stop();                 // stop loader to avoid double feeding
            sleep(300);                           // gap between shots
        }

        robot.shooter.stop();                    // stop shooter


        // ----------- INTAKE SEQUENCE FOR NEXT 3 BALLS -----------
        robot.intake.intakeArtifacts(1.0);
        sleep(2000);                             // collect balls
        robot.intake.stop();

        robot.loader.setLoaderMotor(1.0);        // spacing feed
        sleep(1000);
        robot.loader.stop();

        robot.intake.intakeArtifacts(1.0);       // finish picking up ball #3
        sleep(1500);
        robot.intake.stop();


        // ----------- LAST 3 SHOTS (Shooter first, then everything ON) -----------
        robot.shooter.shootArtifacts();   // rev shooter
        sleep(1200);                      // spin-up time

        for (int i = 0; i < 3; i++) {
            robot.loader.setLoaderMotor(1.0);   // feed ball
            robot.intake.intakeArtifacts(1.0); // push balls forward

            sleep(1100);                       // feed + shoot time

            robot.loader.stop();
            robot.intake.stop();
            sleep(300);
        }

        robot.shooter.stop();
    }
}
