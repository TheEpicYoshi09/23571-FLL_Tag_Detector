package org.firstinspires.ftc.team28420.module;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team28420.types.MovementParams;
import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.Position;
import org.firstinspires.ftc.team28420.types.WheelsRatio;
import org.firstinspires.ftc.team28420.util.Config;

public class Actions {

    private final Movement mv;
    private final IMU imu;
    private final Camera cam;
    private final Shooter shooter;
    private final Servo parkingServo;

    private double cachedHeading = 0.0;

    public Actions(Movement mv, IMU imu, Camera cam, Shooter shooter, Servo parkingServo) {
        this.mv = mv;
        this.imu = imu;
        this.cam = cam;
        this.shooter = shooter;
        this.parkingServo = parkingServo;
    }

    public void init() {
        mv.setup();
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        shooter.setup();
        parkingServo.setPosition(Config.ServoConf.PARKING_SERVO_START_POS);
    }

    public void updateHeading() {
        cachedHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void updateShooter(Gamepad gamepad) {
        if (shooter.state == Shooter.ShooterState.IDLE) {
            if (gamepad.triangle && shooter.manualControl) {
                shooter.snapToNearestSlot();
                shooter.manualControl = false;
            }

            if (gamepad.dpad_left && !shooter.manualRotationActive) {
                shooter.rotateRevolver(-60);
                shooter.curMotif = "";
                shooter.correctMotif = false;
                shooter.manualRotationActive = true;
                shooter.manualControl = true;
            }
            if (gamepad.dpad_right && !shooter.manualRotationActive) {
                shooter.rotateRevolver(60);
                shooter.manualRotationActive = true;
                shooter.manualControl = true;
            }
            if (!gamepad.dpad_right && !gamepad.dpad_left) shooter.manualRotationActive = false;
        }
        if (gamepad.left_bumper) {
            shooter.dribbler.setVelocity(Config.ShooterConf.DRIBBLER_VELOCITY);
        } else shooter.dribbler.setVelocity(0);

        if (gamepad.right_trigger > 0.44) {
            shooter.shooterRun(gamepad.right_trigger);
        } else shooter.shooterStop();


        if (gamepad.right_bumper && shooter.state == Shooter.ShooterState.IDLE && shooter.isShootable()) {
            if (shooter.manualControl ||shooter. correctMotif) {
                shooter. pushBall(true);
                shooter.state = Shooter.ShooterState.SHOOTING;
                shooter.shooterTime.reset();
            }
        }

        // stop shooting state
        if (shooter.state == Shooter.ShooterState.SHOOTING && shooter.shooterTime.milliseconds() >= 2000) {
            shooter.pushBall(false);
            shooter.state = Shooter.ShooterState.STOP_SHOOTING;
            shooter.shooterTime.reset();
        }

        if (shooter.state == Shooter.ShooterState.STOP_SHOOTING && shooter.shooterTime.milliseconds() >= 1000) {
            if (shooter.manualControl) {
                shooter.state = Shooter.ShooterState.IDLE;
            } else {
                shooter.sortedNextBall();
                shooter.state = Shooter.ShooterState.REVOLVER_TURNING;
                shooter.shooterTime.reset();
            }
        }

        if (shooter.state == Shooter.ShooterState.REVOLVER_TURNING && !shooter.revolver.isBusy()) {
            shooter.state = Shooter.ShooterState.IDLE;
            shooter.shooterTime.reset();
        }

        if (!shooter.isShootable() || !shooter.manualControl) shooter.scanBall();
    }

    public void move(WheelsRatio<Double> ratio) {
        updateHeading();
        mv.setMotorsVelocityRatiosWithAcceleration(ratio, Config.WheelBaseConf.MAX_VELOCITY);
    }

    public WheelsRatio<Double> getRatios(double axisX, double axisY, double axisR) {
        return Movement.vectorToRatios(
                PolarVector.fromPos(new Position(axisX, axisY)).rotate(-1 * cachedHeading - Math.PI / 2), axisR);
    }

    public WheelsRatio<Double> getRatiosForApriltag() {
        cam.updateApriltags();
        MovementParams params = cam.getSavedParams();
        return Movement.vectorToRatios(params.getMoveVector(), params.getTurnAbs());
    }

    public void park() {
        parkingServo.setPosition(Config.ServoConf.PARKING_SERVO_STOP_POS);
    }

    public double getCubic(double axis) {
        return Math.pow(axis, 3);
    }

    public double withDeathzone(double axis, double threshold) {
        return Math.abs(axis) < threshold ? 0 : axis;
    }

    public void log() {
        cam.log(Config.Etc.telemetry);
        shooter.log(Config.Etc.telemetry);
    }
}
