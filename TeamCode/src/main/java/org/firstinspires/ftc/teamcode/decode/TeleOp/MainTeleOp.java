package org.firstinspires.ftc.teamcode.decode.TeleOp;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;
// geometery. Pose.getHeading() was on null object refrence
@Config
@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private Robot robot;
    private GamepadEx gp1;


    private PIDController pidController = new PIDController();

    public static PIDGains pidGains = new PIDGains(
            0, 0 ,0
    );


    private boolean isFirst = true;

    private Pose goal = new Pose(136, 136);

    @Override
    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPose(Common.AUTO_END_POSE);
        pidController.setGains(pidGains);

        if(!isRed) goal = goal.mirror();

        waitForStart();


        while (opModeIsActive()) {

            gp1.readButtons();

            if (gamepad1.y && isFirst)  {
                pidController.setTarget(new State(Math.atan2(goal.getY() - robot.drivetrain.getPose().getY(), goal.getX() -  robot.drivetrain.getPose().getX())));
                isFirst = false;
            }

            if (!isFirst) {
                robot.drivetrain.setTeleOpDrive(
                        0,0, pidController.calculate(new State(robot.drivetrain.getHeading())), true
                );
                if (pidController.isInTolerance(new State(robot.drivetrain.getHeading()), Math.toRadians(3))) isFirst = true;
            }

            robot.drivetrain.setTeleOpDrive(
                    -gp1.getLeftY(),
                    gp1.getLeftX(),
                    gp1.getRightX(),
                    true
            );

            double triggerPower =
                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                            - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            robot.shooter.shootArtifacts();

            if (gp1.isDown(GamepadKeys.Button.Y)) {
                robot.loader.setLoaderMotor(1);
            }
            else if (gp1.isDown(GamepadKeys.Button.A)){
                robot.loader.setLoaderMotor(-1);
            }

            if (gp1.isDown(GamepadKeys.Button.X)) {
                robot.intake.intakeArtifacts(0.90);
            } else if (gp1.isDown(GamepadKeys.Button.B)) {
                robot.intake.intakeArtifacts(-.50);
            } else {
                robot.intake.stop();
            }

            robot.run();

            telemetry.addData("Shooter/Loader Power", triggerPower);
            telemetry.update();
        }
    }
}
