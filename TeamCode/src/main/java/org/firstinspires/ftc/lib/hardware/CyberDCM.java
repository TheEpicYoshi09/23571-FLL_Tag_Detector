package org.firstinspires.ftc.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class CyberDCM implements CyberActuator {
    private DcMotorEx dcm;
    double radius = 0.0;

    public CyberDCM(String deviceName) {
        dcm = CyberMap.hardwareMap.get(DcMotorEx.class, deviceName);
        dcm.setTargetPositionTolerance(3);
        //dcm
    }

    @Override
    public void apply(ActuatorConfig config) {
        dcm.setDirection(config.isInverted() ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        dcm.setMode(config.getMode().equals(ActuatorConfig.Mode.FLYWHEEL) ?
                DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
        dcm.setPIDFCoefficients(
                dcm.getMode(),
                new PIDFCoefficients(
                    config.getKP(),
                    config.getKI(),
                    config.getKD(),
                    config.getKF()
                )
        );
        dcm.setZeroPowerBehavior(config.getStopMode().equals(ActuatorConfig.StopMode.BRAKE) ?
                DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        radius = config.getRadius();
    }

    @Override
    public void setSetpoint(Velocity velocity) {
        dcm.setVelocity((velocity.xVeloc / radius), AngleUnit.RADIANS);
    }

    @Override
    public void setSetpoint(AngularVelocity velocity) {
        dcm.setVelocity(velocity.xRotationRate, AngleUnit.RADIANS);
    }

    @Override
    public void setSetpoint(int encoderPosition) {
        dcm.setTargetPosition(encoderPosition);
    }

    @Override
    public void setSetpoint(double proportion) {
        dcm.setPower(proportion);
    }

    @Override
    public int getPosition() {
        return dcm.getCurrentPosition();
    }
}
