package org.firstinspires.ftc.teamcode.RoadRunner.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Decode_2025.DC_Intake_Launch;
import org.firstinspires.ftc.teamcode.Decode_2025.DC_Odometry_Sensor;
import org.firstinspires.ftc.teamcode.Decode_2025.DC_Swerve_Drive;

@TeleOp (name = "_Comp Tank Drive")
public class Comp_TankDrive extends LinearOpMode {

    org.firstinspires.ftc.teamcode.Decode_2025.DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
    org.firstinspires.ftc.teamcode.Decode_2025.DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
    org.firstinspires.ftc.teamcode.Decode_2025.DC_Intake_Launch game = new DC_Intake_Launch(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.SwerveInit();
        odo.DoInit();
        game.InitIL();

        waitForStart();
        drive.dragR.setPosition(.60);
        drive.dragL.setPosition(.55);
        int count = 0;
        while(opModeIsActive()) {
            telemetry.addLine("Driving wheels");
            drive.lfDrive.setPower(-gamepad1.left_stick_y);
            drive.rtDrive.setPower(-gamepad1.right_stick_y);
//            drive.dragL.setPosition(gamepad1.left_stick_x);
//            drive.dragR.setPosition(gamepad1.right_stick_x);
            telemetry.addLine("Left arm: " + drive.dragL.getPosition());
            telemetry.addLine("Right arm: " + drive.dragR.getPosition());
            if(gamepad1.x) {
                drive.dragR.setPosition(.25);
                drive.dragL.setPosition(.75);
            }
            if(gamepad1.y) {
                drive.dragL.setPosition(.55);
                drive.dragR.setPosition(.60);
            }

            //chute setting
            telemetry.addLine("Setting chute");
            game.chute.setPower(-gamepad2.left_stick_y);
            //arm setting
            telemetry.addLine("Setting arm");
            game.arm.setPower(-gamepad2.right_stick_y);
            //launcher setting
            telemetry.addLine("Setting shooter");
            game.launch.setPower(gamepad2.right_stick_x);
            //intake setting
            telemetry.addLine("Setting intake");
            game.intake.setPower(gamepad2.left_stick_x);

            //telemetry update for running
            telemetry.addLine("Count: " + count);
            telemetry.update();
        }
        drive.dragL.setPosition(0);
        drive.dragR.setPosition(0);
    }
}
