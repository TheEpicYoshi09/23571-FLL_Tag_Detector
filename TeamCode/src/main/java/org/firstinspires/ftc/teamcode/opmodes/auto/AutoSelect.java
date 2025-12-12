
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ActiveOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;

@Autonomous(name="AutoSelect", group="Robot", preselectTeleOp = "PyroTeleArcade")
public class AutoSelect extends LinearOpMode
{

    final private ElapsedTime timer = new ElapsedTime();

    ArrayList<Boolean> buttonArray = new ArrayList<>();
    int booleanIncrementer = 0;

    @Override
    public void runOpMode() {

        telemetry.addData(">", "initializing hardware.");
        telemetry.update();
        PinPoint.INSTANCE.init(hardwareMap);
        ActiveOpMode.INSTANCE.init(this);
        Shooter.INSTANCE.init(hardwareMap);
        Lift.INSTANCE.init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Odometry.INSTANCE.teleinit();
        AutoSettings.INSTANCE.readAutoConfig();
        Drive.INSTANCE.init(hardwareMap);
        AutoDrive.INSTANCE.init(telemetry, hardwareMap);
        Vision.INSTANCE.setAlliance(AutoSettings.INSTANCE.iAmBlue());
        telemetry.update();
        telemetry.addData(">", "hardware init complete.");

        AutoSettings.INSTANCE.readAutoConfig();
        telemetry.addData(">", "autoSettings init complete.");
        telemetry.update();

        telemetry.addData(">", "initialization complete.");
        telemetry.update();

        while (!isStarted()) {
            String myAlliance   = AutoSettings.INSTANCE.iAmBlue() ? "Blue" : "Red";
            String myPosition   = AutoSettings.INSTANCE.atGoal() ? "AtGoal" : "AtWall";


            boolean g1xPressed = ifPressed(gamepad1.x);
            boolean g1dpdPressed = ifPressed(gamepad1.dpad_down);
            boolean g1dpuPressed = ifPressed(gamepad1.dpad_up);
            boolean g1dplPressed = ifPressed(gamepad1.dpad_left);
            boolean g1dprPressed = ifPressed(gamepad1.dpad_right);
            boolean g1backPressed = ifPressed(gamepad1.back);

            if (g1dpuPressed) { AutoSettings.INSTANCE.I_AM_BLUE = !AutoSettings.INSTANCE.I_AM_BLUE; }
            if (g1dplPressed) { AutoSettings.INSTANCE.AT_GOAL = !AutoSettings.INSTANCE.AT_GOAL; }
            // if (g1dplPressed) { incTurnPos(); }

            if (booleanIncrementer != 0) {
                AutoSettings.INSTANCE.saveAutoConfig();
                Vision.INSTANCE.setAlliance(AutoSettings.INSTANCE.I_AM_BLUE);
                //backdropPixel.update(autoSettings.iAmBlue());
                //placePixel.update(autoSettings.iAmBlue());
                //doubleVision.update(autoSettings.iAmBlue(), autoSettings.rightShift(), autoSettings.leftShift());
                //odometry.init(autoSettings.iAmBlue(), autoSettings.iAmBackstage());
            }
            booleanIncrementer = 0;

            telemetry.addData("Alliance Color (Dpad Up): ", myAlliance);
            telemetry.addData("Start Position (Dpad Left): ", myPosition);
            telemetry.addData("Obelisk Tag ID: ", Vision.INSTANCE.getObeliskTag());
            telemetry.addLine(" ");

            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            sleep(20);
        }

        timer.reset();
        //while (opModeIsActive() & timer.seconds() < 3.) { }

        if (AutoSettings.INSTANCE.AT_GOAL) {
            AtGoal.runTest();
        } else {
            AtWall.runTest();
        }
        //backdropPixel.update(autoSettings.iAmBlue());
        //doubleVision.update(autoSettings.iAmBlue(), autoSettings.rightShift(), autoSettings.leftShift());
        //lastPosition = doubleVision.MarkerProcessor.getPosition();

        //FieldPosition fieldPosition = FieldPosition.BACKSTAGE;
        //if (! autoSettings.iAmBackstage()) fieldPosition = FieldPosition.FRONTSTAGE;
        //TargetTurnPosition turnPosition = autoSettings.myTurnPosition();

        //if (autoSettings.goPixel()) {
        //    placePixel.goPushPixelAndReturn(lastPosition,
        //            turnPosition,
        //            fieldPosition,
        //            autoSettings.rightShift(),
        //            autoSettings.leftShift());
       // }

        telemetry.addData("Robot Pos x/y ", "%7f/%7f", Odometry.INSTANCE.getX(),
                           Odometry.INSTANCE.getY());
        telemetry.addData("Robot heading deg ", "%7f", Odometry.INSTANCE.getHeading());
        telemetry.update();
        //sleep(5000);
    }

    /*
    private void incTurnPos() {
        if (autoSettings.TURN_POSITION == TargetTurnPosition.NULL ) {autoSettings.TURN_POSITION = TargetTurnPosition.BACKSTAGE; return;}
        if (autoSettings.TURN_POSITION == TargetTurnPosition.BACKSTAGE ) {autoSettings.TURN_POSITION = TargetTurnPosition.FRONTSTAGE; return;}
        if (autoSettings.TURN_POSITION == TargetTurnPosition.FRONTSTAGE ) {autoSettings.TURN_POSITION = TargetTurnPosition.GATE; return;}
        if (autoSettings.TURN_POSITION == TargetTurnPosition.GATE ) {autoSettings.TURN_POSITION = TargetTurnPosition.PERIMETER; return;}
        if (autoSettings.TURN_POSITION == TargetTurnPosition.PERIMETER ) {autoSettings.TURN_POSITION = TargetTurnPosition.BACKSTAGE;}
    }
     */

    private boolean ifPressed(boolean button) {
        boolean output = false;
        boolean buttonWas;
        if (buttonArray.size() == booleanIncrementer) {
            buttonArray.add(false);
        }
        buttonWas = buttonArray.get(booleanIncrementer);
        if (button != buttonWas && buttonWas) {
            output = true;
        }
        buttonArray.set(booleanIncrementer, button);
        booleanIncrementer += 1;
        return output;
    }

}