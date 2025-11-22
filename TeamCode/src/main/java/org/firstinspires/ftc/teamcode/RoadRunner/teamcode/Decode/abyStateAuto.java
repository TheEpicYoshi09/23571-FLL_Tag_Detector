package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Autonomous", group = "Swerve")
public class abyStateAuto extends LinearOpMode {

  enum State {
    Find,
    Approach,
    Aim,
    Shoot,
    Exit,
    Done
  }

  public State currentState = State.Shoot;
  public Telemetry telemetry;
  // get classes required
  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  // yep do the same
  DC_Intake_Launch game = new DC_Intake_Launch(this);
  DC_BallSensor present = new DC_BallSensor(this);
  DC_Husky_Sensor ball = new DC_Husky_Sensor(this);
  DC_Husky_Sensor april = new DC_Husky_Sensor(this);

  @Override
  public void runOpMode() {
    // initialize components
    drive.SwerveInit();
    game.InitIL();
    present.SensorInit();
    ball.initHuskyLens();
    ball.setAlgorithm("color");
    april.initHuskyLens();
    april.setAlgorithm("april");
    // Wait for the DS start button to be touched.
    telemetry.addLine("Ready");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      // run state machine
      run();
    }
    // exit autonomous
  }

  public void run() {

    switch (currentState) {
      case Find:
        telemetry.addLine("State Machine: Find");
        // do stuff in here

        currentState = State.Approach;
        break;

      case Approach:
        telemetry.addLine("State Machine: Approach");
        // do stuff in here

        currentState = State.Aim;
        break;

      case Aim:
        telemetry.addLine("State Machine: Aim");
        // do stuff in here

        currentState = State.Shoot;
        break;
      case Shoot:
        telemetry.addLine("State Machine: Shoot");
        while (game.spinUp(5000)) {
          idle();
        }
       // decode.setGate(1.0); // open
        // set timer
        while (present.Present()) {
          // wait for ball
          idle();
        }
       // decode.setGate(0.0); // close
        game.armPosition(1);
        while (!present.Present()) {
          idle();
        }
        game.armPosition(0);
        game.spinOff();
        currentState = State.Find;
        break;
      case Exit:
        telemetry.addLine("State Machine: Complete");
        // do stuff in here
        if (opModeIsActive()) {
          currentState = State.Find;
        } else return;
        break;

      default:
        telemetry.addLine("State Machine: error");
        break;
    } // state machine
  } // run
} // autonomous
