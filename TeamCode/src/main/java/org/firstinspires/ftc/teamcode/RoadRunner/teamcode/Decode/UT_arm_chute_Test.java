package org.firstinspires.ftc.teamcode.RoadRunner.teamcode.Decode;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_arm_chute_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  @Override
  public void runOpMode() {

    game.InitIL();
    double armVolt = 0.0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;

    waitForStart();

    while (opModeIsActive()) {
        telemetry.addLine("Move with caution ");
        telemetry.addLine("Observe Limit and position voltages");
        telemetry.addLine("then set values into DC_Intake_Launch");
        telemetry.addLine("- - - - - - - - - - - - - - - - - - - -" );
        telemetry.addLine("Arm");
        telemetry.addLine("- - - - - - - - - -" );
        game.armPosition(-gamepad1.left_stick_y);
        telemetry.addData("Potentiometer Value", game.getArmPot());
        telemetry.update();
        telemetry.addLine("Chute");
        telemetry.addLine("- - - - - - - - - -" );
        game.chutePosition(-gamepad1.right_stick_y);
        telemetry.addData("Potentiometer Value", game.getChutePot());
        telemetry.update();
    }
  }
}
