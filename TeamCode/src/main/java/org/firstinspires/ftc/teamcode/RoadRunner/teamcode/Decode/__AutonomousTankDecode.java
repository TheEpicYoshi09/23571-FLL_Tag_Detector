package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

/*
 *-* Control configuration
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class __AutonomousTankDecode extends LinearOpMode {

  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  DC_Swerve_Drive robot = new DC_Swerve_Drive(this);
  DC_Intake_Launch game = new DC_Intake_Launch(this);
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  DC_Husky_Sensor husk = new DC_Husky_Sensor(this);

  @Override
  public void runOpMode() {

    // initialize all the hardware, using the hardware class. See how clean and simple this is?
    robot.SwerveInit();
    game.InitIL();
    husk.initHuskyLens();
    husk.setAlgorithm("color");

    double topDrive = 0.0;
    double leftDrive = 0.0;
    // Send telemetry message to signify robot waiting;
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    // run until the end of the match (driver presses STOP)
    if(opModeIsActive()) {
      robot.servodrag(.65);
      game.spinUp(6000.0);
      // ball 1 away
      game.armPosition(.6);// determine position arm shoot
      sleep(500);
      game.armPosition(.8);// arm up
      game.Intake(1.0);// intake next ball into chute
      sleep(1000); // wait for ball
      game.Intake(0.0); // stop intake
      // ball 2 away
      game.armPosition(.6);// determine position arm shoot
      sleep(500);
      game.armPosition(.8);// arm up
      game.Intake(1.0);// intake next ball into chute
      sleep(1000); // wait for ball
      // ball 3 away
      game.armPosition(.6);// determine position arm shoot
      sleep(500);
      game.spinOff();
      game.armPosition(.8);// arm up
      robot.lfDrive.setPower(.4);
      robot.rtDrive.setPower(.4);
      sleep(1000);
      robot.lfDrive.setPower(.0);
      robot.rtDrive.setPower(.0);

      // robot.gamePadTeleOp();
      husk.setBlocks(); // get the largest object

      telemetry.addData("Id", husk.getObjId());
      topDrive = husk.topPos();
      leftDrive = husk.leftPos(); // 0 - 1

      }
      telemetry.addLine("  field location");
      telemetry.addLine(" *****************");
      odo.ppo.getPosition();
      odo.ppo.update();
      telemetry.addData("position X", odo.ppo.getPosX(DistanceUnit.INCH));
      telemetry.addData("position Y", odo.ppo.getPosY(DistanceUnit.INCH));

      telemetry.update();
    } // end autonomous run loop

    // sleep(500);

} // end class Swerve Automation
