package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.SixBallsAutoPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SixBallsFullAuto", group = "Autos")
public class SixBallsFullAuto extends OpMode {

    private Follower follower;
    private Robot robot;

    private PathChain path;
    private Timer timer;
    private int state;

    private final Pose startPose = new Pose(88.122, 7.597, Math.toRadians(90));

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        path = SixBallsAutoPath.build(follower);

        timer = new Timer();
        state = 0;
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.followPath(path);
        state = 0;
    }

    @Override
    public void loop() {

        follower.update();

        switch (state) {

            // 🟦 STATE 0 — wait for path to finish
            case 0:
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    state = 1;
                }
                break;

            // 🟦 STATE 1 — Shot #1 (rev first, then load)
            case 1:
                robot.shooter.shootArtifacts();     // rev shooter
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    robot.loader.setLoaderMotor(1.0);
                }
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    robot.loader.stop();
                    timer.resetTimer();
                    state = 2;
                }
                break;

            // 🟦 STATE 2 — Shot #2
            case 2:
                robot.shooter.shootArtifacts();
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    robot.loader.setLoaderMotor(1.0);
                }
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    robot.loader.stop();
                    timer.resetTimer();
                    state = 3;
                }
                break;

            // 🟦 STATE 3 — Shot #3
            case 3:
                robot.shooter.shootArtifacts();
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    robot.loader.setLoaderMotor(1.0);
                }
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    robot.loader.stop();
                    robot.shooter.stop();
                    timer.resetTimer();
                    state = 4;
                }
                break;

            // 🟦 STATE 4 — loader bump (make space)
            case 4:
                robot.loader.setLoaderMotor(1.0);
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    robot.loader.stop();
                    timer.resetTimer();
                    state = 5;
                }
                break;

            // 🟦 STATE 5 — intake new rings
            case 5:
                robot.intake.intakeArtifacts(1.0);
                if (timer.getElapsedTimeSeconds() > 1.0) {
                    robot.intake.stop();
                    timer.resetTimer();
                    state = 6;
                }
                break;

            // 🟦 STATE 6 — load into hopper (no shooting yet)
            case 6:
                robot.loader.setLoaderMotor(1.0);
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    robot.loader.stop();
                    timer.resetTimer();
                    state = 7;
                }
                break;

            // 🟦 STATE 7 — Final shot sequence (rev → load)
            case 7:
                robot.shooter.shootArtifacts();
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    robot.loader.setLoaderMotor(1.0);
                }
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    robot.loader.stop();
                    robot.shooter.stop();
                    state = -1; // done
                }
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
