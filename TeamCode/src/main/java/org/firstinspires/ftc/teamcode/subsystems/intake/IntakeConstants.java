package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeConstants {
    public static final String kPivotMasterId = "20";
    public static final String kPivotSlaveId = "21";
    public static final String kFlywheelId = "22";

    public static final boolean kPivotInverted = false;
    public static final Motor.ZeroPowerBehavior kPivotIdleMode
            = Motor.ZeroPowerBehavior.BRAKE;
    public static final double kPivotP = (1.0 / 3.0);
    public static final int kPivotTolerance = 13;

    public static final boolean kFlywheelsInverted = false;
    public static final Motor.ZeroPowerBehavior kFlywheelIdleMode
            = Motor.ZeroPowerBehavior.FLOAT;
    public static final double kFlywheelP = 0.5;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelS = 0.03;
    public static final double kFlywheelV = 0.1;
}
