package org.firstinspires.ftc.team28420.module;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team28420.types.PolarVector;
import org.firstinspires.ftc.team28420.types.WheelsRatio;
import org.firstinspires.ftc.team28420.util.Config;

public class Movement {

    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private double currentLF = 0.0;
    private double currentRF = 0.0;
    private double currentLB = 0.0;
    private double currentRB = 0.0;

    public Movement(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    public void setMotorsTargetPosition(WheelsRatio<Double> wheelsRatio) {
        WheelsRatio<Integer> wheelsRatioInteger = wheelsRatio.toInt(1);
        leftFront.setTargetPosition(wheelsRatioInteger.getLeftTop());
        rightFront.setTargetPosition(wheelsRatioInteger.getRightTop());
        leftBack.setTargetPosition(wheelsRatioInteger.getLeftBottom());
        rightBack.setTargetPosition(wheelsRatioInteger.getRightBottom());
    }

    public void setMotorsPowerRatios(WheelsRatio<Double> wheelsRatio) {
        Config.Etc.telemetry.addData("left front", wheelsRatio.getLeftTop());
        Config.Etc.telemetry.addData("right front", wheelsRatio.getRightTop());
        Config.Etc.telemetry.addData("left bottom", wheelsRatio.getLeftBottom());
        Config.Etc.telemetry.addData("right bottom", wheelsRatio.getRightBottom());


        leftFront.setPower(wheelsRatio.getLeftTop());
        rightFront.setPower(wheelsRatio.getRightTop());
        leftBack.setPower(wheelsRatio.getLeftBottom());
        rightBack.setPower(wheelsRatio.getRightBottom());
    }

    public void setMotorsVelocityRatios(WheelsRatio<Double> wheelsRatio, int velocityMult) {
        currentLF = slewRate(currentLF, wheelsRatio.getLeftTop() * velocityMult, Config.WheelBaseConf.MAX_ACCEL);
        currentRF = slewRate(currentRF, wheelsRatio.getRightTop() * velocityMult, Config.WheelBaseConf.MAX_ACCEL);
        currentLB = slewRate(currentLB, wheelsRatio.getLeftBottom() * velocityMult, Config.WheelBaseConf.MAX_ACCEL);
        currentRB = slewRate(currentRB, wheelsRatio.getRightBottom() * velocityMult, Config.WheelBaseConf.MAX_ACCEL);

        leftFront.setVelocity(currentLF);
        rightFront.setVelocity(currentRF);
        leftBack.setVelocity(currentLB);
        rightBack.setVelocity(currentRB);
    }

    public double slewRate(double current, double target, double max_accel) {
        double step = target - current;
        double clampedStep = Range.clip(step, -max_accel, max_accel);
        return current + clampedStep;
    }


    public void setMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }

    public void brake() {
        setMotorsPowerRatios(WheelsRatio.ZERO);
    }

    public void setup() {
        //leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        //leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public WheelsRatio vectorToRatios(PolarVector vector, double turn) {
        double sin = Math.sin(vector.getTheta() - Math.PI/4);
        double cos = Math.cos(vector.getTheta() - Math.PI/4);
        Config.Etc.telemetry.addData("theta: ", vector.getTheta() * 180.0/Math.PI);
        //double sin = Math.sin(vector.getTheta() + Math.PI / 4);
        //double cos = Math.cos(vector.getTheta() + Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double lf = vector.getAbs() * cos / max + turn;
        double rf = vector.getAbs() * sin / max - turn;
        double lb = vector.getAbs() * sin / max + turn;
        double rb = vector.getAbs() * cos / max - turn;

        if ((vector.getAbs() + Math.abs(turn)) > 1) {
            lf /= vector.getAbs() + Math.abs(turn);
            rf /= vector.getAbs() + Math.abs(turn);
            lb /= vector.getAbs() + Math.abs(turn);
            rb /= vector.getAbs() + Math.abs(turn);
        }

        return new WheelsRatio<Double>(lf, rf, lb, rb);
    }

}
