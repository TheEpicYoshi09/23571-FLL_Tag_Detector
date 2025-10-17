package org.firstinspires.ftc.teamcode.testing;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "SpindexerTest", group = "Teleop")
public class SpindexerTester extends LinearOpMode {
    ColorSensor colorSensor;
    Indexer indexer;
    GamepadEx gp2;

    @Override
    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        indexer = new Indexer(hardwareMap);
        gp2 = new GamepadEx(gamepad2);

        waitForStart();
        while(opModeIsActive()){
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                indexer.nextState();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.A))
            {
                indexer.setIntaking(!indexer.getIntaking());
            }
            telemetry.addData("Light Detected:",((OpticalDistanceSensor) colorSensor).getLightDetected());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());

            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            String mainColor = getMainColor(r, g, b);
            telemetry.addData("Color Detected overall:",mainColor);

            telemetry.update();
        }
    }

    @NonNull
    private static String getMainColor(int r, int g, int b) {
        double total = r+g+b;
        double rRatio = r / total;
        double gRatio = g / total;
        double bRatio = b / total;

        String mainColor;

        if (rRatio > 0.45 && gRatio < 0.35 && bRatio < 0.35) {
            mainColor = "Red";
        } else if (rRatio > 0.45 && gRatio > 0.25 && bRatio < 0.20) {
            mainColor = "Orange";
        } else if (rRatio > 0.38 && gRatio > 0.38 && bRatio < 0.25) {
            mainColor = "Yellow";
        } else if (gRatio > 0.45 && rRatio < 0.35 && bRatio < 0.35) {
            mainColor = "Green";
        } else if (bRatio > 0.45 && rRatio < 0.35 && gRatio < 0.35) {
            mainColor = "Blue";
        } else if (rRatio > 0.35 && bRatio > 0.35 && gRatio < 0.30) {
            mainColor = "Purple";
        } else {
            mainColor = "Unclear";
        }
        return mainColor;
    }
}
