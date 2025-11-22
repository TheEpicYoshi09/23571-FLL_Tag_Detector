package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "_Tank TeleOp", group = "Decode")
public class __TeleOpTankDecode extends LinearOpMode {
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  DC_Intake_Launch game = new DC_Intake_Launch(this);
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  double gpLy = 0.0;
  double gpRy = 0.0;
  double gpLx = 0.0;
  double gpRx = 0.0;
  double gpLt = 0.0;
  double gpRt = 0.0;

  @Override
  public void runOpMode() {
      drive.SwerveInit();
      game.InitIL();
      odo.DoInit();
      waitForStart();
      drive.dragL.setPosition(.8);
      drive.dragR.setPosition(-.8);
      int count = 0;
      while (opModeIsActive()) {
        gpLt = gamepad1.left_trigger;
        gpLy = -gamepad1.left_stick_y;
        gpRt = gamepad1.right_trigger;
        gpRy = -gamepad1.right_stick_y;

        telemetry.addData("RY",gpRy);
        telemetry.addData("LY",gpLy);
        telemetry.addLine(count + "");
        count++;
        telemetry.update();
        drive.lfDrive.setPower(gpLy);
        drive.rtDrive.setPower(gpRy);
        // set arm to position
        game.armPosition(gamepad2.left_stick_y);
        // set chute to position
        game.chutePosition(gamepad2.right_stick_y);
        if (gamepad2.a) {
          // Start spin set velocity as needed
          boolean spinup = game.spinUp(5000);
        }
        if (gamepad2.b) {
          // Stop spin
          game.spinOff();
        }
        if (gamepad2.y) {
          // Stop spin
          drive.dragL.setPosition(.1);
          drive.dragR.setPosition(-.1);
        }
        if (gamepad2.left_bumper) {
          // Start Intake
          game.Intake(1.0);//start Intake - may need to set direction -1 - 1
        }
        if (gamepad2.right_bumper) {
          // Stop Intake
          game.Intake(0.0);//stop Intake
        }


        // scaleRange(min, max) configure the following servo control by trigger
        // to straf drive to finish alignment to shoot: by April tag
        // check angle of wheel if not 180 then
        /*gpLt = (1.0 - gamepad1.left_trigger)/2.0;
        gpRt = 0.5 - (gamepad1.right_trigger)/2.0;
        if (gpLt < 0.5){
          // turn left by servo
          drive.lfTurn.setPosition(gpLt);
          drive.rtTurn.setPosition(0.5);
        } else {
          // wheels lock 180/0
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(0.5);
        }
        if (gpRt > 0.5){
          // turn right by servo
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(gpRt);
        } else {
          // wheels lock 180/0
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(0.5);
        }*/
        // determine if 90 degrees then turn and lock left
        // determine if -90 degrees then turn and lock right
        //drive wheels

      }// while op Mode
  }// run Op mode
}// tank TeleOp
