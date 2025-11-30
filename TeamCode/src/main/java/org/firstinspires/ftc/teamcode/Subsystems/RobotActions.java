package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class RobotActions {

    private static Robot robot;

    public RobotActions(Robot robotInstance) {
        robot = robotInstance;
    }

    // INTAKE ----------------------------------------------
    public static Action intakeAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.intakeArtifacts(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }

    // LOADER ----------------------------------------------
    public static Action loaderAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.loader.setLoaderMotor(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.loader.stop())
        );
    }

    // SHOOTER ---------------------------------------------
    public static Action shooterAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.shooter.shootArtifacts()),   // FIXED: this actually runs motors
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.shooter.stop())
        );
    }

    // STOP EVERYTHING -------------------------------------
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.loader.stop();
            robot.shooter.stop();
        });
    }

    // FULL TEST SEQUENCE ---------------------------------
    public static Action fullSequence() {
        return new SequentialAction(
                intakeAction(1.0, 5),
                loaderAction(1.0, 5),
                shooterAction(1.0, 5),
                stopAll()
        );
    }
}
