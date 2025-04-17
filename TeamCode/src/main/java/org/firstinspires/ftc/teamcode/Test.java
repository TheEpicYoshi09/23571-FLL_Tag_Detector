// /* Copyright (c) 2017 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.Range;

// /**
//  * This file provides basic Telop driving for a Pushbot robot.
//  * The code is structured as an Iterative OpMode
//  *
//  * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
//  * All device access is managed through the HardwarePushbot class.
//  *
//  * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
//  * It raises and lowers the claw using the Gampad Y and A buttons respectively.
//  * It also opens and closes the claws slowly using the left and right Bumper buttons.
//  *
//  * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
//  */

// @TeleOp(name="test", group="Pushbot")
// public class Test extends OpMode
// {
//     /* Declare OpMode members. */
//     HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
//     double          clawOffset  = 0.0 ;                  // Servo mid position
//     final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    
//     final double ARM_UP_POWER = 0.7;
//     final double ARM_DOWN_POWER = -0.7;
      
//     final double VERTICAL_UP_POWER = 20;
//     final double VERTICAL_DOWN_POWER= -20;
    
//     final double ROTATE_UP_POWER = 0.;
//     final double ROTATE_DOWN_POWER = -1;

//     String armStatus = "";
    
//     final int rotateDownPosition = -100;
//     final int rotateUpPosition = 0;
    
//     final int verticalDownPosition = 0;
//     final int verticalUpPosition = 1350;
    
//     //for distance sensor
//     final float OPTIMAL_DISTANCE = 5; 
//     double DISTANCE;

//     /*
//      * Code to run ONCE when the driver hits INIT
//      */
//     //@Override
//     public void init() 
//     {
//         /* Initialize the hardware variables.
//          * The init() method of the hardware class does all the work here
//          */
//         robot.init(hardwareMap);

//         // Send telemetry message to signify robot waiting;
//         telemetry.addData("Say", "" + "hi Driver");
//     }

//     /*
//      * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//      */
//     //@Override
//     public void init_loop() 
//     {
//     }

//     /*
//      * Code to run ONCE when the driver hits PLAY
//      */
//     //@Override
//     public void start() 
//     {
//     }

//     /*
//      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//      */
//     // @Override
//     public void loop() 
//     {
        
//         double forward = - gamepad1.left_stick_y;
//         double strafe = gamepad1.left_stick_x;
//         double turn = gamepad1.right_stick_x;
        
//         robot.leftFrontDriveWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//         robot.leftFrontDriveWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
//         //Controls the speed for going across the field; can be changed if going too slow
//             //strafe is lateral movement!!! Looking forward while moving sideways
//             //if(gamepad1.right_bumper)
//             //{
//               //  forward /= 4;  
//              //   strafe /= 4;
//              //   turn /= 4;
//             //}
            

//         // //Use gamepad left & right Bumpers to open and close the claw
//         // if (gamepad2.right_bumper)
//         //     clawOffset += CLAW_SPEED;
//         // else if (gamepad2.left_bumper)
//         //     clawOffset -= CLAW_SPEED;
         
//          //hanging contraption
//         //if (gamepad1.dpad_down)
//          //{
//              //robot.verticalMotor.setPower(VERTICAL_DOWN_POWER);
//          //}
//          //else if (gamepad1.dpad_up)
//          //{
//           //   robot.verticalMotor.setPower(VERTICAL_UP_POWER);
           
//         // }
//         //else
//         //{
//             //setting power to 0 so that when buttons arent being held down it dosent move!!
//             //robot.verticalMotor.setPower(0);
//         //}
        
//         //for claw servo
        
//         //for rotating arm
//         if(gamepad2.dpad_up)
//         {
//             robot.leftFrontDriveWheel.setTargetPosition(rotateUpPosition);
//             robot.leftFrontDriveWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//             robot.leftFrontDriveWheel.setPower(ROTATE_UP_POWER);

//         }
//         else if(gamepad2.dpad_down)
//         {
//             robot.leftFrontDriveWheel.setTargetPosition(rotateDownPosition);
//             robot.leftFrontDriveWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//             robot.leftFrontDriveWheel.setPower(ROTATE_DOWN_POWER);
            
//             telemetry.addData("encoder thing", robot.leftFrontDriveWheel.getCurrentPosition());
//             telemetry.update();
//         }
        
       
//       DISTANCE=robot.distanceSensor.getDistance(DistanceUnit.CM);
//       telemetry.addData("currently at", DISTANCE);
//       telemetry.addData("position:", robot.clawServo.getPosition());
//       telemetry.update();
       
//       if(DISTANCE == OPTIMAL_DISTANCE)
//       {
//         telemetry.addData("at optimal distance, press A for arm movement",DISTANCE);
//         telemetry.update();
       
//         if(gamepad1.a)
//         {
//           //arm stuff
//         }
//       }
//     }

//     /*
//      * Code to run ONCE after the driver hits STOP
//      */
//         @Override
//      public void stop() 
//      {
         
//      }
//   }
