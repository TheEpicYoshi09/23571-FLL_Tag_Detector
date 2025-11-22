package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;
// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class DC_Intake_Launch {
  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode myOp = null;

  // Default constructor
  public DC_Intake_Launch(LinearOpMode opmode) {
    myOp = opmode;
      present = new DC_BallSensor(myOp);
  }

  public DC_BallSensor present;

  public DcMotorEx launch = null; // 6000 rpm motor
  public DcMotor arm = null; // 312 rpm motor
  public CRServo chute = null;
  public CRServo intake = null; // intake motor controller
  //private Servo gate = null; // intake motor controller
  private AnalogInput armpot = null; // Arm Potentiometer input ( Axon pot)
  private AnalogInput chutepot = null; // chute Potentiometer input ( Axon pot)
  private double armMin = 0.2;
  private double armMax = 3.05;
  private double chuteMin = 0.2;
  private double chuteMax = 3.05;
  // status light
  private final RevBlinkinLedDriver status = null;
  private final RevBlinkinLedDriver.BlinkinPattern pattern = null;
  private Deadline ledCycleDeadline;

  DisplayKind displayKind;

  protected enum DisplayKind {
    MANUAL,
    AUTO
  }

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD); // todo set launch direction
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm = myOp.hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD); // todo set arm direction
    chute = myOp.hardwareMap.get(CRServo.class, "chute");
    // Define and Initialize Servo
    intake = myOp.hardwareMap.get(CRServo.class, "intake");
    //gate = myOp.hardwareMap.get(Servo.class, "gate");
    armpot = myOp.hardwareMap.get(AnalogInput.class, "armPot");
    present.SensorInit();
    //status = myOp.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    //status.setPattern(pattern);
  }

  // Servo controlled motor
  public void Intake(double FS) {
    intake.setPower(FS); // front intake servo motor
  }
  /* ball gate
  public void setGate(double FS) {
    gate.setPosition(FS);
  }*/
  // the following are motor checks only
  public void startLaunch(){
    launch.setPower(1.0);
  }
  public void stopLaunch(){
    launch.setPower(0.0);
  }
  public void startArm(){
    arm.setPower(1.0);
  }
  public void stopArm(){
    arm.setPower(0.0);
  }
  public void launchVelocity(){
    launch.setVelocity(1000.0);
    myOp.sleep(2000);
    myOp.telemetry.addData("Current Velocity", launch.getVelocity());
    myOp.telemetry.update();
    launch.setVelocity(0.0);
  }
  // end motor checks

  public boolean spinUp(double shoot_velocity) {
    boolean ready = true; // the not of false
    launch.setVelocity(shoot_velocity);
    while (myOp.opModeIsActive() && ready) {
      myOp.telemetry.addData("Current Velocity", launch.getVelocity());
      myOp.telemetry.update();
      if (launch.getVelocity() > shoot_velocity) ready = false;
    }
    // set status light
    return ready;
  } // end spin up

  public void spinOff() {
    launch.setVelocity(0.0);
  }

  public double getArmPot(){
    return armpot.getVoltage();
  }

  public double getChutePot(){
    return chutepot.getVoltage();
  }

  public boolean armPosition(double pos) {
    // pos is gamepad joystick or -1 to 1
    // analog voltage
    double armPwr = 0.3;
    double potVolt = mapPotenToServo();
    double armMinH = 0.3; // min arm position
    double armMaxH = 0.8; // max arm position
    double toServoPos = (pos + 1.0)/2.0;
    if(toServoPos > armMinH) {
      if (toServoPos < armMaxH) {
        do {
          if (potVolt < toServoPos) arm.setPower(armPwr); // increase
          if (potVolt >= toServoPos) arm.setPower(-armPwr);
        } // decrease
        while (myOp.opModeIsActive() && Math.abs((toServoPos - potVolt)) > .1);
          arm.setPower(0.0);
      }// is less
    }// is more
    return true;
  } // end arm position

  public boolean chutePosition(double pos) {
    // pos is gamepad joystick or -1 to 1
    double chutePwr = .3;
    // analog voltage
    double potVolt = mapPotenToServo();
    double chuteMinH = 0.3; // min arm position
    double chuteMaxH = 0.8; // max arm position
    double toServoPos = (pos + 1.0)/2.0;
      // is more
      if (toServoPos > chuteMinH && toServoPos < chuteMaxH) {
          do {
              if (potVolt < toServoPos) chute.setPower(chutePwr); // increase
              if (potVolt >= toServoPos) chute.setPower(-chutePwr);
          } // decrease
          while (myOp.opModeIsActive() && Math.abs((toServoPos - potVolt)) > .1);
          arm.setPower(0.0);
      }// is less
      return true;

  } // end chute position

  private double mapPotenToServo(){
    /**
     * Maps the current analog feedback voltage to a proportional 0.0-1.0 position value.
     *
     * @return The current servo position as a double between 0.0 and 1.0.
     */
      double CV = getArmPot();
      // Ensure the voltage is within the expected range before mapping
      CV = Math.max(armMin, Math.min(armMax, CV));

      // Apply the linear interpolation formula
      double mappedPosition = (CV - armMin) /
              (armMax - armMin);

      return mappedPosition;
    }// map servo poteniometer voltage to servo position
  }
