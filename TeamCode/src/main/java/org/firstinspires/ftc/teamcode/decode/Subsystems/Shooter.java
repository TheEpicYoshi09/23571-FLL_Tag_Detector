package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private MotorEx shooter;
    private MotorEx followerShooter;

    public Shooter(HardwareMap hardwareMap) {
        shooter = new MotorEx(hardwareMap, "leftShooterMotor", Motor.GoBILDA.BARE);
        followerShooter = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);
    }

    public void shootArtifacts() {
        shooter.set(1);
        followerShooter.set(1);
    }

    public void velocityShooter(){

    }

    public void stop() {
        shooter.set(0);
        followerShooter.set(0);
    }
}
