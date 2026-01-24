package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Random;

//Shooter class
public class Shooter {
    private final DcMotor shooterMotor;
    //Shooters starting power constants
    public double shooterPower = Constants.shooterPower;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.shooterMotor = hardwareMap.dcMotor.get(Constants.shooterMotor());
        this.shooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    //Shooter power increase function
    public void increasePower() {
        this.shooterPower = shooterPower + 0.1;
        this.shooterPower = Math.min(shooterPower, 1.0);
    }

    //Shooter power decrease function
    public void decreasePower() {
        this.shooterPower = shooterPower - 0.1;
        this.shooterPower = Math.max(shooterPower, 0);
    }

    //Constant for shooter motors stop
    public void setZero(DcMotor motor) {
        motor.setPower(0);
    }

    public void setMotorPower(double power) {
        this.shooterMotor.setPower(power);
    }

   //Return shooter motors and powers
    public DcMotor getShooterMotor() {
        return this.shooterMotor;
    }
    public double getShooterPower() {
        return this.shooterPower;
    }

    public void shooter100() {
        this.shooterPower = 1;
        setMotorPower(this.shooterPower);
    }

    public void shooter30() {
        this.shooterPower = 0.3;
        setMotorPower(this.shooterPower);
    }

    public void shooter50() {
        this.shooterPower = 0.5;
        setMotorPower(this.shooterPower);
    }

    public void shooter70() {
        this.shooterPower = 0.7;
        setMotorPower(this.shooterPower);
    }

    public void shooter(float power) {
        this.shooterPower = power;
        setMotorPower(this.shooterPower);
    }

    public void randomShooter() {
        Random rand = new Random();
        int n = rand.nextInt(100) + 1;
        shooter(n);
    }

    public void reverseMotor() {
        this.shooterPower = -1;
        setMotorPower(this.shooterPower);
    }

    public double shooterEncoderPosition() {
        return this.shooterMotor.getCurrentPosition();
    }
}
