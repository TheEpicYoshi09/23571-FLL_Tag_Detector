//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "DecodeTeleopV2.99 Alaqmar", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Chassis chassis;
    private volatile boolean threadIsRunning = true;
    private Thread driveThread;


    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis();
        chassis.init(this);
//   chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Intake intake =new Intake();
        intake.init(this);

        Kicker kicker = new Kicker();
        kicker.init(hardwareMap);

        DecodeAprilTag aprilTag  = new DecodeAprilTag(this);
        aprilTag.initCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        chassis.odo.resetPosAndIMU();

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task

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



            //Running the following code in Thread
                float axial = -gamepad1.left_stick_y;
                float lateral = -gamepad1.left_stick_x;
                float yaw = -gamepad1.right_stick_x;
                chassis.moveRobot(axial, lateral, yaw);


            // Kicker
            double gateClose = 0.7;
            double gateShooting = 0.4;
            double gateIntake = 1;
            long flyWheelReadyTime = 1000;

            if(gamepad2.dpad_down) {
                kicker.setKickerPos(gateClose);
            }
            if(gamepad2.dpad_up) {
                kicker.setKickerPos(gateShooting);
            }
            if(gamepad2.dpad_right) {
                kicker.setKickerPos(gateIntake);
            }
            //Shooting
            if (gamepad2.right_bumper) {

                intake.intake(0.6); 
                kicker.setKickerPos(gateClose);// Middle P
                sleep(1000);

                long startTime = System.currentTimeMillis();

                flyWheel.setPower(-0.65);

                telemetry.addData("Flywheel start power: ",  + flyWheel.getPower());

                while (flyWheel.getVelocity() < -1500){}

                long endTime = System.currentTimeMillis();
                long durationInMillis = endTime - startTime;

                telemetry.addData("Flywheel warmup time (ms): ",  + durationInMillis );

                double currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before First Shot", currentVelocity);


                sleep(1000);
                // First Shot
                kicker.setKickerPos(gateShooting);
                sleep(500);
                kicker.setKickerPos(gateClose);
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity After First Shot", currentVelocity);

                // Turn intake on
                sleep(flyWheelReadyTime);
                intake.intake(0.6);
                sleep(200);
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before 2nd Shot", currentVelocity);

                while (flyWheel.getVelocity() < -1500){}

                //Second Shot
                kicker.setKickerPos(gateShooting);
                sleep(500);
                kicker.setKickerPos(gateClose);
                sleep(flyWheelReadyTime);

                while (flyWheel.getVelocity() < -1500){}

                // Third Shot
                currentVelocity = flyWheel.getVelocity();
                telemetry.addData("Velocity Before Third Shot", currentVelocity);
                kicker.setKickerPos(gateShooting);

                telemetry.update();
                sleep(5000);

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
                intake.intake(1);
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

        // Clean up the thread
        threadIsRunning = false;
        driveThread.interrupt();
        try {
            driveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        }

    private class DriveTask implements Runnable {
        @Override
        public void run() {
            while (threadIsRunning && !Thread.currentThread().isInterrupted()) {

                // Read gamepad input and set drive motor power
                float axial = -gamepad1.left_stick_y;
                float lateral = -gamepad1.left_stick_x;
                float yaw = -gamepad1.right_stick_x;
                chassis.moveRobot(axial, lateral, yaw);

                try {
                    Thread.sleep(10); // Pause for a short time
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
}
