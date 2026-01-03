package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
@TeleOp
public class MecanumLauncher extends OurOpmode{

    Launcher launcher;
    MecanumDrive drive;
    IMUW imu;

    @Override
    protected void Loop() {
        drive.driveVectorField(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,0.5 + (gamepad1.left_bumper? 0.2 : 0),imu);

        launcher.launchTeleOp(gamepad1.rightBumperWasPressed());
        
        if (gamepad1.bWasPressed()){
            launcher.stopFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Stopped");
        }
        
        if (gamepad1.yWasPressed()){
            launcher.spinUpFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Spinning Up");
        }

        logVariables();
    }

    private void logVariables() {
        logger.logData(Logger.LoggerMode.STATUS,"FlyWheelVelocity",launcher.getLauncher().getVelocity());
        logger.logData("TargetVelocity",launcher.getLauncherTargetVelocity());
        logger.logData(Logger.LoggerMode.STATUS,"Launch-state",launcher.getLaunchStatesT());
        logger.logData("Gampad1 LeftstickY",gamepad1.left_stick_y);
        logger.logData("Gampad1 Leftstickx",gamepad1.left_stick_x);
        logger.logData("Gampad1 RightstickX",gamepad1.right_stick_x);
        logger.update();
    }

    @Override
    protected void initialize() {
        logger = new Logger(telemetry);
        launcher = new Launcher(this);
        drive = new MecanumDrive(this,logger, MecanumDrive.RobotName.BOB);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        drive.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.resetYaw();
        logger.logData(Logger.LoggerMode.CRITICAL,"Status","Init");
        logger.update();
    }
}
