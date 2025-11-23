
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

import java.util.ArrayList;
import java.util.List;

//@Disabled 
@Autonomous

public class Test extends LinearOpMode {
    
    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry);
        
        // April Tag Detection ------------------------------------
        
        // Make the list of ball pattern
        List<String> pattern1 = new ArrayList<>();
        pattern1.add("Green");
        pattern1.add("Purple");
        
        
         // Create the AprilTag reader object
        AprilTagReader tagReader = new AprilTagReader(hardwareMap);

        int detectedTag = -1;

        // Read tags during INIT
        while (opModeInInit()) {
            detectedTag = tagReader.getTagId();
            telemetry.addData("Detected Tag", detectedTag);
            telemetry.update();
        }
        
        

        waitForStart();
            robot.shooter.startShot(1, "short");
        
      

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Detected Tag", detectedTag);
            telemetry.update();

        }
    }
}
