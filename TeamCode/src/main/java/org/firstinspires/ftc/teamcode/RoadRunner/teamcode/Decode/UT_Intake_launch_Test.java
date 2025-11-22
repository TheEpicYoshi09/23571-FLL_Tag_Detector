package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Intake_launch_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  private final DC_BallSensor Ball = new DC_BallSensor(this);
  private final DC_Swerve_Drive Drv = new DC_Swerve_Drive(this);
  private final DC_Husky_Sensor Seek = new DC_Husky_Sensor(this);

  @Override
  public void runOpMode() {

    game.InitIL();
    Ball.SensorInit();

    waitForStart();

    if (opModeIsActive()) {
      // start intake system
      double moveLR = 0.0;
      double moveFB = 0.0;
      game.Intake(1.0);
      game.startLaunch();
      sleep(500);
      game.stopLaunch();
      game.startArm();
      sleep(500);
      game.stopArm();
      game.launchVelocity();
      while (opModeIsActive() && Ball.Present()) {
        // GId ball found
        if (Seek.GId() > 0) {
          moveLR = Seek.getLeft(); // may need neg. of
          moveFB = Seek.getTop() - 1.0; // move back picking ball
        }
      }
      //spin up
      while (opModeIsActive() && game.spinUp(200)) {
        idle();
      }
      sleep(1000);
      game.Intake(.5);
      game.spinOff();
      //
      game.Intake(1.0);
      while (opModeIsActive() && game.spinUp(400)) {
        idle();
      }
      sleep(3000);

      game.Intake(.5);
     game.spinOff();
    }
  }
}
