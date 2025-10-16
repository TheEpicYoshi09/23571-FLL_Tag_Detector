//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;

@TeleOp(name = "DecodeTeleopV2.94 Alaqmar", group = "TeleOp")

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis();
        chassis.init(this);
//   chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Intake intake =new Intake();
        intake.init(this);

        Kicker kicker = new Kicker();
        kicker.init(hardwareMap);

//        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        chassis.odo.resetPosAndIMU();
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            chassis.odo.update();
            telemetry.addData("Odo x", chassis.odo.getEncoderX());
            telemetry.addData("Odo y", chassis.odo.getEncoderY());
            telemetry.update();
//            if (gamepad1.a) {
//                chassis.resetHeading();
//
//            }
//            // If gamepad x is pressed then switch to field centric
//             If gamepad y is pressed then switch to robot centric
//            if (gamepad1.x) {
//                chassis.resetIMU();
//                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
//            } else if (gamepad1.y) {
//                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
//            }

            float axial = -gamepad1.left_stick_y;
            float lateral = -gamepad1.left_stick_x;
            float yaw = -gamepad1.right_stick_x;
            chassis.moveRobot(axial, lateral, yaw);

            // Kicker
            double gateClose = 0.4;
            double gateShooting = 0.05;
            double gateIntake = 0.8;
            long flyWheelReadyTime = 600;
            //Shooting
            if (gamepad2.right_bumper) {
                intake.stopIntake();
                kicker.setKickerPos(gateClose);// Middle P
                sleep(500);
                flyWheel.start(-0.58);
                sleep(1800);

                // First Shot
                kicker.setKickerPos(gateShooting);
                sleep(200);
                kicker.setKickerPos(gateClose);

                // Turn intake on
                sleep(flyWheelReadyTime);
                intake.intake(0.8);
                sleep(200);

                //Second Shot
                kicker.setKickerPos(gateShooting);
                sleep(200);
                kicker.setKickerPos(gateClose);
                sleep(flyWheelReadyTime);

                // Third Shot
                kicker.setKickerPos(gateShooting);

              // Turns Flywheel off.
            } else if (gamepad2.left_bumper) {
                intake.stopIntake();
                kicker.setKickerPos(gateClose);
                flyWheel.start(1);
                sleep(530);
                flyWheel.stop();
                kicker.setKickerPos(gateIntake);


        }
            if (gamepad2.a){
                intake.intake(0.5);
            } else if (gamepad2.b) {
                intake.stopIntake();
            }

          /*  if (gamepad2.x){
                kicker.setKickerPos(1);
            }
            else if (gamepad2.y)
            kicker.setKickerPos(0.5);

            else if (gamepad2.dpad_right){
                kicker.setKickerPos(0.15);
            }*/
                }
        }

}
