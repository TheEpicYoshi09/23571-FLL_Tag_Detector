/* Copyright (c) 2017 FIRST. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided that
* the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list
* of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice, this
* list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* Neither the name of FIRST nor the names of its contributors may be used to endorse or
* promote products derived from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
* This file provides basic Telop driving for a Pushbot robot.
* The code is structured as an Iterative OpMode
*
* This OpMode uses the common Pushbot hardware class to define the devices on the robot.
* All device access is managed through the HardwarePushbot class.
*
* This particular OpMode executes a basic Tank Drive Teleop for a PushBot
* It raises and lowers the claw using the Gampad Y and A buttons respectively.
* It also opens and closes the claws slowly using the left and right Bumper buttons.
*
* Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
* Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class PushbotTeleopTank_Iterative extends OpMode
{
    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
      
    final int ARM_UP_POWER = 100;
    final int ARM_DOWN_POWER= -100;
    
    final double ROTATE_UP_POWER = 2;
    final double ROTATE_DOWN_POWER = -2;
    
    final double otherRotatePowerUp = 2;
    final double otherRotatePowerDown = -2;
    
    final double otherArmPowerUp = 2;
    final double otherArmPowerDown = 2;
    
    String armStatus = "";
    
    final int armDownPosition = 0;
    final int armUpPosition = 1350;
    
    final int rotateDownPosition = 430;
    final int rotateMidPosition = 200;
    final int rotateUpPosition = 0;
    
    //for distance sensor
    final float OPTIMAL_DISTANCE = 5; 
    double DISTANCE;

    /*
    * Code to run ONCE when the driver hits INIT
    */
    //@Override
    public void init() 
    {
        /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
        robot.init(hardwareMap);
        
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        robot.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotateMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //these 2 lines were added to fix the "set my position pls" error thing
        robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition());
        robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition());

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "" + "hi Driver");
        telemetry.addData("encoder thing arm", robot.armMotor.getCurrentPosition());
        telemetry.addData("encoder thing rotate", robot.rotateMotor.getCurrentPosition());
    }

    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
    //@Override
    public void init_loop() 
    {
    }

    /*
    * Code to run ONCE when the driver hits PLAY
    */
    //@Override
    public void start() 
    {
    }

    /*
    * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    */
    // @Override
    public void loop() 
    {
        
        double forward = - gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        //Controls the speed for going across the field; can be changed if going too slow
            //strafe is lateral movement!!! Looking forward while moving sideways
            //if(gamepad1.right_bumper)
            //{
              //  forward /= 4;  
            //   strafe /= 4;
            //   turn /= 4;
            //}
            
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
            
        robot.rightFrontDriveWheel.setPower((forward - strafe - turn) / denominator);
        robot.leftFrontDriveWheel.setPower((forward + strafe + turn) / denominator);
        robot.leftBackDriveWheel.setPower((forward - strafe + turn) / denominator);
        robot.rightBackDriveWheel.setPower((forward + strafe - turn) / denominator);

        // //Use gamepad left & right Bumpers to open and close the claw
        // if (gamepad2.right_bumper)
        //     clawOffset += CLAW_SPEED;
        // else if (gamepad2.left_bumper)
        //     clawOffset -= CLAW_SPEED;


        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad2.y)
        {
            robot.armMotor.setTargetPosition(armUpPosition);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(ARM_UP_POWER);
            
            //telemetry.addData("encoder thing arm", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad2.a) //&& robot.touchSensor.isPressed())
        {
            robot.armMotor.setTargetPosition(armDownPosition);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(ARM_DOWN_POWER);
            
            //telemetry.addData("encoder thing arm", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        //else
        //{
            //robot.armMotor.setPower(0);
        //}
       if(gamepad2.right_stick_y > 0.05) //&& robot.rotateMotor.getCurrentPosition() > 0 ) //rotate up
        {
            //2 lines below are meg's idea of how to run joystick teleop
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotor.setTargetPosition(armDownPosition);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(gamepad2.right_stick_y/2);
        
            //telemetry.addData("gamepad", gamepad2.left_stick_y);
            //telemetry.addData("zero power", robot.rotateMotor.getZeroPowerBehavior());
            //telemetry.update();
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           // robot.rotateMotor.setPower(gamepad2.left_stick_y);
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // robot.rotateMotor.setPower(0.02);
            // int stuff = (int)(Math.ceil((double)rotateDownPosition * gamepad2.left_stick_y));
            // robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition() - stuff);
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // telemetry.addData("stuff", stuff);
            //telemetry.update();
        //     robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.rotateMotor.setPower(ROTATE_DOWN_POWER);
        //   // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //     telemetry.addData("stick is working", gamepad2.left_stick_y);
        //     telemetry.update();
            //
            
            //robot.rotateMotor.setPower(gamepad2.left_stick_y);
            //robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition());
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //two lines above and this line below were how it origionally was working last
            //robot.rotateMotor.setPower(otherRotatePowerUp);
        }
        else if(gamepad2.right_stick_y<-0.05)
        {
            robot.armMotor.setTargetPosition(armUpPosition);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(gamepad2.right_stick_y/2);
        }
        else if(gamepad2.right_stick_y<0.05 && gamepad2.right_stick_y>-0.05)
        {
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setTargetPosition(robot.armMotor.getTargetPosition());
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           //robot.rotateMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
           robot.armMotor.setPower(0);
        }
         
        //hanging contraption
        //if (gamepad1.dpad_down)
        //{
            //robot.verticalMotor.setPower(VERTICAL_DOWN_POWER);
        //}
        //else if (gamepad1.dpad_up)
        //{
          //   robot.verticalMotor.setPower(VERTICAL_UP_POWER);
           
        // }
        //else
        //{
            //setting power to 0 so that when buttons arent being held down it dosent move!!
            //robot.verticalMotor.setPower(0);
        //}
        
        //for claw servo
        if(gamepad1.right_bumper)
        {
          robot.clawServo.setPosition(0.6);
        }
        else if(gamepad1.left_bumper)
        {
          robot.clawServo.setPosition(0.0);
        }

        //for rotating arm
        if(gamepad2.dpad_up)
        {
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rotateMotor.setTargetPosition(rotateUpPosition);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMotor.setPower(ROTATE_UP_POWER);

        }
        else if(gamepad2.dpad_down)
        {
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rotateMotor.setTargetPosition(rotateDownPosition);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMotor.setPower(ROTATE_DOWN_POWER);
        }
        else if(gamepad2.dpad_right)
        {
            robot.rotateMotor.setTargetPosition(rotateMidPosition);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMotor.setPower(ROTATE_DOWN_POWER);
        }
        
        telemetry.addData("gamepad", gamepad2.left_stick_y);
        telemetry.update();
        
        if(gamepad2.left_stick_y < -0.05) //&& robot.rotateMotor.getCurrentPosition() > 0 ) //rotate up
        {
            //2 lines below are meg's idea of how to run joystick teleop
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rotateMotor.setTargetPosition(0);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMotor.setPower(gamepad2.left_stick_y/2);
        
            telemetry.addData("gamepad", gamepad2.left_stick_y);
            telemetry.addData("zero power", robot.rotateMotor.getZeroPowerBehavior());
            telemetry.update();
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           // robot.rotateMotor.setPower(gamepad2.left_stick_y);
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // robot.rotateMotor.setPower(0.02);
            // int stuff = (int)(Math.ceil((double)rotateDownPosition * gamepad2.left_stick_y));
            // robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition() - stuff);
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // telemetry.addData("stuff", stuff);
            //telemetry.update();
        //     robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.rotateMotor.setPower(ROTATE_DOWN_POWER);
        //   // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //     telemetry.addData("stick is working", gamepad2.left_stick_y);
        //     telemetry.update();
            //
            
            //robot.rotateMotor.setPower(gamepad2.left_stick_y);
            //robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition());
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //two lines above and this line below were how it origionally was working last
            //robot.rotateMotor.setPower(otherRotatePowerUp);
        }
        else if(gamepad2.left_stick_y>0.05)
        {
            robot.rotateMotor.setTargetPosition(rotateDownPosition);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMotor.setPower(gamepad2.left_stick_y/2);
        }
        else if(gamepad2.left_stick_y<0.05 && gamepad2.left_stick_y>-0.05)
        {
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rotateMotor.setTargetPosition(robot.rotateMotor.getTargetPosition());
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           //robot.rotateMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
           robot.rotateMotor.setPower(0);
        }
        
        // else if(gamepad2.left_stick_y < 0) //&& robot.rotateMotor.getCurrentPosition() < 430) //rotate down 
        // {
        //     robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //     robot.rotateMotor.setPower(gamepad2.left_stick_y);
            
        //     telemetry.addData("gamepad", gamepad2.left_stick_y);
        //     telemetry.update();
            
            //robot.rotateMotor.setPower(gamepad2.left_stick_y);
            //int stuff = (int)(Math.ceil((double)rotateDownPosition * gamepad2.left_stick_y));
            //robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition() + stuff);
           // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //telemetry.addData("stuff", stuff);
            //telemetry.update();
            
           // robot.rotateMotor.setPower(gamepad2.left_stick_y);
        //    int stuff = (int)(Math.ceil((double)rotateDownPosition * gamepad2.left_stick_y));
          //  robot.rotateMotor.setTargetPosition(stuff);
            //telemetry.addData("stuff", stuff);
            //telemetry.update();
            
            
            
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.rotateMotor.setPower(gamepad2.left_stick_y);
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rotateMotor.setTargetPosition(robot.rotateMotor.getCurrentPosition());
            //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rotateMotor.setPower(otherRotatePowerDown);
            
            
        //   // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //     //robot.rotateMotor.setPower(gamepad2.left_stick_y/2);
        //     //robot.rotateMotor.setTargetPosition(rotateUpPosition * gamepad2.left_stick_y);
        //     //robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     //robot.rotateMotor.setPower(ROTATE_DOWN_POWER); 
        //   // robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
         
    
      //DISTANCE=robot.distanceSensor.getDistance(DistanceUnit.CM);
      //telemetry.addData("currently at", DISTANCE);
      //telemetry.addData("position:", robot.clawServo.getPosition());
      //telemetry.update();
       
    //   if(DISTANCE == OPTIMAL_DISTANCE)
    //   {
    //     telemetry.addData("at optimal distance, press A for arm movement",DISTANCE);
    //     telemetry.update();
       
    //     if(gamepad1.a)
    //     {
    //       //arm stuff
    //     }
    //   }
    
        telemetry.addData("encoder thing rotate", robot.rotateMotor.getCurrentPosition());
        telemetry.update();
    }

    /*
    * Code to run ONCE after the driver hits STOP
    */
        @Override
    public void stop() 
    {
        robot.rotateMotor.setTargetPosition(rotateUpPosition);
        robot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotateMotor.setPower(ROTATE_UP_POWER);
    }
  }
