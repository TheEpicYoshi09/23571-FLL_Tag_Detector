package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;




//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "TeleOp", group = "Driver") //The name and group
@Configurable
@Disabled
public class Teleop extends OpMode {

    public static int obeliskApriltagID = 21;

    final double angleOfLauncherInDegrees = 35;
    final double heightOfRobotInMeters = 0.35;
    final double heightToGoalWithClearance = (1.11125) - (heightOfRobotInMeters);
    final double turretPulleyRatio = (double) 24 /155; //Motor to Turret
    final double turretTicksPerDegree = 537.7/360;

    double distanceToGoalInMeters = 0.0;

    Follower follower;

    DcMotorEx active;
    DcMotorEx launcher;
    DcMotorEx rotary;

    int rotaryCurrentPosition = 0;
    int rotaryTargetPosition = 0;

    public static double pR, dR, lR, fR;
    PDFLController rotaryController = new PDFLController(pR, dR, fR, lR);

    public static double pL, dL, lL, fL;
    PDFLController launcherController = new PDFLController(pL, dL, fL, lL);
    public static double launcherTargetPosition = 0;

    double turretTargetAngle = 0;


    ArrayList<UniConstants.slotState> slots = new ArrayList<>(List.of(UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY));
    ArrayList<UniConstants.slotState> pattern = new ArrayList<>();
    ArrayList<ColorSensor> colorSensors = new ArrayList<>();


    CRServo stupidServo;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        active = hardwareMap.get(DcMotorEx.class, UniConstants.ACTIVE_INTAKE_STRING);
        active.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcher = hardwareMap.get(DcMotorEx.class, UniConstants.LAUNCHER_STRING);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotary = hardwareMap.get(DcMotorEx.class, UniConstants.ROTARY_STRING);

        rotary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotary.setDirection(UniConstants.ROTARY_DIRECTION);

        colorSensors.addAll(
                List.of(
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_FRONT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING))
                )
        );




        obeliskTargetPattern(obeliskApriltagID);
    }

    @Override
    public void start() {
        for (ColorSensor sensor : colorSensors) {
            sensor.enableLed(true);
        }




    }

    @Override
    public void loop() {


        if (gamepad1.a && !allFull()) {
            active.setPower(.5);
            if (isFull(slots.get(0))) {
                rotaryTargetPosition += UniConstants.SPACE_BETWEEN_ROTARY_SLOTS;
            }
        }


        readSlots();



        //If blue, aim for blue goal else aim for red goal - this target position will be fed to servo
        //Is it possible to send data from auto to tele? like whether blue or not
        //Think about only doing this every 500ms or so so we arent constantly computing
        //Tradeoff of accuracy vs chub efficiency

        //Velocity should be constantly interpolated via apriltag localization data right? Heavy comp power
        //Velocity should only be queried when ready to shoot.
        //TODO: Generate linear regression to determine velocity for given positions
        //Exit velocity will be a function of the power put into the motor (PDFL)


        //Slot numbers are already known, 0 is ready to transfer into launcher, 1 is to the right, 2 is the last one.
        //So for obelisk PPG, 0 = P, 1 = P, 2 = G
        //If that is good, bool readyToLaunch = true
        //I need the robot to be built or cad to be done to actually do this without theory coding but shrug
        rotaryController.setTarget(rotaryTargetPosition);
        rotaryCurrentPosition = rotary.getCurrentPosition();
        rotaryController.update(rotaryCurrentPosition);
        rotary.setPower(rotaryController.runPDFL(5));


        launcherController.setTarget(launcherTargetPosition);
        launcherController.update(launcher.getCurrentPosition());
        launcher.setPower(launcherController.runPDFL(5));

        distanceToGoalInMeters = getDistanceToGoalInMeters();
        turretTargetAngle = 0; //Vision.getTargetAngle

        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

        telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
        telemetry.addData("Slot 0 State ", slots.get(0));
        telemetry.addData("Slot 1 State ", slots.get(1));
        telemetry.addData("Slot 2 State ", slots.get(2));
        telemetry.addLine();
        telemetry.addData("Launcher Target Position ", launcherTargetPosition);
        telemetry.addData("Launcher Target Angle (Deg) ", turretTargetAngle);
        telemetry.addData("Calculated Launch Velocity ", getTargetVelocity(distanceToGoalInMeters));
        telemetry.addData("Distance To Goal In Meters ", distanceToGoalInMeters);
        telemetry.addLine();



        telemetry.update();


    }

    public void readSlots() {
        for (int i = 0; i < 3; i++) {
            double hue = colorSensors.get(i).argb();
            if ((UniConstants.PURPLE_ARTIFACT_LOWER_HUE < hue) && (hue < UniConstants.PURPLE_ARTIFACT_UPPER_HUE)) {
                slots.set(i, UniConstants.slotState.PURPLE);
            } else if ((UniConstants.GREEN_ARTIFACT_LOWER_HUE < hue) && (hue < UniConstants.GREEN_ARTIFACT_UPPER_HUE)) {
                slots.set(i, UniConstants.slotState.GREEN);
            } else {
                slots.set(i, UniConstants.slotState.EMPTY);
            }
        }

    }

    public double getTargetVelocity(double distanceToGoalInMeters) {
        //https://www.desmos.com/calculator/yw7iis7m3w
        //https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
        return Math.sqrt(
                ((9.81) * (Math.pow(distanceToGoalInMeters, 2)))
                        /
                        (Math.pow(2 * (Math.cos(Math.toRadians(angleOfLauncherInDegrees))), 2) * ((distanceToGoalInMeters * Math.tan(Math.toRadians(angleOfLauncherInDegrees))) - heightToGoalWithClearance))
        );
    }

    public boolean isFull(UniConstants.slotState slot) {
        return (slot == UniConstants.slotState.PURPLE) || (slot == UniConstants.slotState.GREEN);
    }

    public boolean allFull() {

        for (UniConstants.slotState slot : slots) {
            if (!isFull(slot)) {
                return false;
            }
        }
        return true;
    }

    public void obeliskTargetPattern(int ID){
        switch(ID){
            case 21:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE));
                break;
            case 22:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE));
                break;
            case 23:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN));
                break;
        }

    }


    public double getDistanceToGoalInMeters() {

        double x = follower.getPose().getX() - Poses.blueGoal.getX();
        double y = follower.getPose().getY() - Poses.blueGoal.getY();
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))) / 39.37;



    }

    public double getTurretTargetPosition(double turretTargetAngle, double ratio, double ticksPerDegree){

        //Ratio given in terms of motor/turret

        return turretTargetAngle * ratio * ticksPerDegree;

    }

}
