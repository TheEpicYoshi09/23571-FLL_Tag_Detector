package org.firstinspires.ftc.teamcode.decode.TeleOp;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp
public class ServoTeleOp extends OpMode{

        HoodServo servo = new HoodServo();
        @Override
        public void init(){
            servo.init(hardwareMap);
        }
        @Override
        public void loop(){
            if (gamepad1.a){
                servo.setHoodservo(0);
            }
            else if (gamepad1.x){
                servo.setHoodservo(1);
            }

        }

}
