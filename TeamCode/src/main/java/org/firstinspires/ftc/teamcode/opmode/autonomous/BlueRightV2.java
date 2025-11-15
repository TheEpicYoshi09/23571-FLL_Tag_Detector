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

package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name=" Auto Drive: Right Side", group="Robot")
@Disabled
public class BlueRightV2 extends AutoCommon {

    @Override
    public void runOpMode() {

        initAll();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
        encoderDrive(0.2,  5,  5, 5.0);
        strafe_encoder(0.4, 55, 55, 5.0);
        encoderDrive(0.3,  10,  -10, 5.0);
        putSamplesInHighBacket();

         */
        // strafe to right
        //tilt.setPosition(BotCoefficients.tiltUp);
        //driveAndHangSpeciman();
        //getSpecimen();
        //driveAndHangSpeciman();
        //simpleParkRight();
        //pushSamples();
        //simpleParkLeft();
        //back a little bit

        //encoderDrive(0.5,  60,  60, 5.0);

        /*
        turnToTargetYaw(-90, 0.4, 2000);
        encoderDrive(0.2,  20,  20, 5.0);
        touchLowBar();

         */
        sleep(5000);
        // drive forward
        //encoderDrive(0.2,  31,  31, 5.0);
        // strafe to right
        //strafe_encoder(0.4, 20, 20, 5.0);
        // drive backward to push sample into parking area
        //encoderDrive(0.2,  -60,  -60, 5.0);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        //grabber.setPosition(0);
        // Step 1:  Drive forward for 3 seconds
        //String line = detectTeamPropLine("blue far");
        //visionPortal.close();
        //line = "left";
/*
        if (line.equals("middle")) {
            //driveToMiddleLine();
            encoderDrive(0.2,  31,  31, 5.0);
            //strafe(0.2, 1000);
            dropPixelOnLine();
            encoderDrive(0.4,  -15,  -15, 5.0);
            turnToTargetYaw(-20+yaw0, 0.4, 5000);
            encoderDrive(0.5,   50, 50, 4.0);

            turnToTargetYaw4(-90+yaw0, 0.4, 3000);
            encoderDrive(0.5,   -80, -80, 4.0);

            turnToTargetYaw4(-60+yaw0, 0.4, 3000);
            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -60,  -60, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(700);
            //shiftLeft(0.2, 2000);
            //turn(-0.5, 0.3,1260);
            //turnToTargetYaw(110+yaw0, 0.4, 5000);
            //encoderDrive(0.2,  -5,  -5, 5.0);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //encoderDrive(0.4,  -75,  -75, 5.0);
            //turnToTargetYaw(45+yaw0, 0.4, 5000);
            //encoderDrive(0.4,  -35,  -35, 5.0);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //encoderDrive(0.2,  -30,  -30, 5.0);
            //turnToTargetYaw(-90+yaw0, 0.4, 5000);
            //encoderDrive(0.5,   , -6, 4.0);
            //driveToBackBoardNearSide(2);
            //encoderDrive(0.2,  -50,  -50, 5.0);
            //backward(0.2, 5000);
            //dropPixelOnBoard();
        }
        else if (line.equals("right")) {
            encoderDrive(0.2,  27,  27, 5.0);
            turnToTargetYaw(-20+yaw0, 0.2, 5000);
            //encoderDrive(0.2,  22,  22, 5.0);

            dropPixelOnLine();
            encoderDrive(0.5,  -15,  -15, 5.0);

            turnToTargetYaw4(yaw0, 1.0, 4000);
            //driveToRightLine();
            encoderDrive(0.5,  45,  45, 5.0);
            //turn(0.1, -0.5,1000);
            turnToTargetYaw4(-90+yaw0, 1.0, 3000);
            encoderDrive(0.5,  -80,  -80, 5.0);
            turnToTargetYaw4(-60+yaw0, 1.0, 3000);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -50,  -50, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(700);
            //driveToLeftLine();
            //backward(0.2, 2500);

            //encoderDrive(0.2,  -48,  -48, 5.0);
            //turnToTargetYaw(yaw0, 0.4, 5000);
            //encoderDrive(0.2,  -25,  -25, 5.0);
            //turnToTargetYaw(50+yaw0, 0.4, 5000);
            //turn(-0.2, 0.2,800);
            //encoderDrive(0.2,  -10,  -10, 5.0);
            //sleep(100);
            //encoderDrive(0.2,  13,  13, 5.0);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //dropPixelOnLine();
            //forward(0.2, 800);
            //turn(-0.5, 0.3, 1000);
            //driveToBackBoardNearSide(1);
            //backward(0.2, 4500);
            //encoderDrive(0.2,  -48,-48, 5.0);
            //dropPixelOnBoard();
        }
        else {
            encoderDrive(0.2,  26,  26, 5.0);
            turnToTargetYaw(55+yaw0, 0.2, 5000);
            //encoderDrive(0.2,  38,  38, 5.0);
            //turnToTargetYaw2(-45+yaw0, 0.2, 5000);
            encoderDrive(0.2,  6,  6, 5.0);
            dropPixelOnLine();
            encoderDrive(0.5,  -10,  -10, 5.0);
            turnToTargetYaw4(yaw0, 1.0, 5000);
            //driveToRightLine();
            encoderDrive(0.5,  40,  40, 5.0);
            //turn(0.1, -0.5,1000);
            turnToTargetYaw4(-90+yaw0, 1.0, 3000);
            encoderDrive(0.5,  -80,  -80, 5.0);
            turnToTargetYaw4(-60+yaw0, 1.0, 3000);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -50,  -50, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(700);

            //turn(0.1, -0.5,1000);
            //turnToTargetYaw(45+yaw0, 0.4, 5000);
            //encoderDrive(0.3,  -20,  -20, 5.0);
            //sleep(100);
            //encoderDrive(0.3,  37,  37, 5.0);
            //sleep(100);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //sleep(100);
            //encoderDrive(0.3,  -28,  -28, 5.0);
            //sleep(100);
            //turnToTargetYaw(-90+yaw0, 0.4, 5000);
            //dropPixelOnLine();
            //forward(0.2, 2500);
            //turn(-0.4, 0.1, 2200);
            //driveToBackBoardNearSide(3);
            //backward(0.2, 4500);
            //encoderDrive(0.2,  -38,  -38, 5.0);
            //encoderDrive(0.4,  -85,  -85, 5.0);
            //turnToTargetYaw(45+yaw0, 0.4, 5000);
            //encoderDrive(0.4,  -30,  -30, 5.0);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //encoderDrive(0.2,  -30,  -30, 5.0);
            //dropPixelOnBoard();
        }
        visionPortal.close();
        */

    }
}
