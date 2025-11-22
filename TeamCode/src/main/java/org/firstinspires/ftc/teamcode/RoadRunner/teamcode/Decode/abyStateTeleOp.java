package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOp", group = "Swerve")
public class abyStateTeleOp extends LinearOpMode {

  enum State {
    Loop,
    Find,
    Approach,
    Aim,
    Shoot,
    Exit,
    Done
  }
  int ctBalls = 0;
  public State currentState = State.Loop;
  public Telemetry telemetry;

  @Override
  public void runOpMode() {

    // Wait for the DS start button to be touched.
    telemetry.addLine("Ready");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      // run state machine
      run(); //
    }
    // exit autonomous
  }

  // get classes required
  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  DC_Swerve_Drive robot = new DC_Swerve_Drive(this);
  // yep do the same
  DC_Intake_Launch decode = new DC_Intake_Launch(this);
  DC_Husky_Sensor seeker = new DC_Husky_Sensor(this);
  DC_BallSensor present = new DC_BallSensor(this);

  public void run() {

    switch (currentState) {
      case Loop:
        // drive component
        // keep track of balls
        // opMode running ball max not early launch
       /* while (opModeIsActive() && ctBalls < 4  && !dDown()){

        } */
        // game component
        if(dUp()){currentState = State.Find;}
        if(dLeft()){currentState = State.Aim;}
        if(dDown()){currentState = State.Shoot;}
        if(!opModeIsActive()){currentState = State.Exit;}
        telemetry.update();
        break;
      case Find:
        telemetry.addLine("State Machine: Find");
        // read camera for objects of color
        seeker.getObjId();
        // if found signal
        // Approach states if found then get ball
        currentState = State.Approach;
        break;

      case Approach:
        telemetry.addLine("State Machine: Approach");
        // start intake to capture ball
        decode.Intake(1.0);// full intake speed
          // then when top > 220 check for another ball that's 2
          // after 2 3 prepare to launch balls
          double lr = seeker.getLeft();
          // set left right adjust to swerve
          //if(lr > .55)
          //if(lr < .45)
          // set backup to swerve
          double bkup = seeker.getTop();
          // set speed to back up to ball
          //? did I get a ball ?
          // open gate and receive ball
          //
        currentState = State.Loop;
        break;

      case Aim:
        telemetry.addLine("State Machine: Aim");
        // do stuff in here

        currentState = State.Loop;
        break;
      case Shoot:
        telemetry.addLine("State Machine: Shoot");
        while (decode.spinUp(5000)) {
          idle();
        }
        //decode.setGate(1.0); // open
        // set timer
        while (present.Present()) {
          // wait for ball
          idle();
        }
        //decode.setGate(0.0); // close
        decode.armPosition(1);
        while (!present.Present()) {
          idle();
        }
        decode.armPosition(0);
        decode.spinOff();
        currentState = State.Find;
        break;
      case Exit:
        telemetry.addLine("State Machine: Complete");
        // do stuff in here
        if (opModeIsActive()) {
          currentState = State.Loop;
        } else return;
        break;

      default:
        telemetry.addLine("State Machine: error");
        break;
    } // state machine
  } // run
  private boolean dDown(){
    boolean done = false;
    while(gamepad2.dpad_down){
      sleep(2);
      if(gamepad2.dpad_down) done = true;
    }
    return done;
  }
  private boolean dUp(){
    boolean done = false;
    while(gamepad2.dpad_up){
      sleep(2);
      if(gamepad2.dpad_up) done = true;
    }
    return done;
  }
  private boolean dLeft(){
    boolean done = false;
    while(gamepad2.dpad_left){
      sleep(2);
      if(gamepad2.dpad_left) done = true;
    }
    return done;
  }
  private boolean dRight(){
    boolean done = false;
    while(gamepad2.dpad_right){
      sleep(2);
      if(gamepad2.dpad_right) done = true;
    }
    return done;
  }
} // TeleOp
