package org.firstinspires.ftc.teamcode.Util;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class UniConstants {

//CONFIG
    //CHM0 = ACTIVE
    //CHM1 = TURR
    //CHM2 = FLM
    //CHM3 = BLM
    //CHI2C0 = imu
    //CHI2C1 = CSF
    //CHI2C2 = CSL
    //CHI2C3 = pp


    //EHM0 = FR
    //EHM1 = BR
    //EHM2 = ROTA
    //EHM3 = LAUNCH
    //EHI2C0 = CSR
    //EHS5 = BS



    //Drive
    public static final String DRIVE_FRONT_LEFT_STRING = "FLM";
    public static final String DRIVE_FRONT_RIGHT_STRING = "FRM";
    public static final String DRIVE_BACK_LEFT_STRING = "BLM";
    public static final String DRIVE_BACK_RIGHT_STRING = "BRM";
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.FORWARD;

    public enum loggingState{
        DISABLED,
        ENABLED,
        EXTREME
    }

    public enum slotState{
        PURPLE,
        GREEN,
        EMPTY,
        BETWEEN
    }

    public enum teamColor {
        RED,
        BLUE
    }

    public static final String COLOR_SENSOR_SLOT_FRONT_STRING = "CSF";
    public static final String COLOR_SENSOR_SLOT_RIGHT_STRING = "CSR";
    public static final String COLOR_SENSOR_SLOT_LEFT_STRING = "CSL";

    public static final String ACTIVE_INTAKE_STRING = "ACTIVE";
    public static final String LAUNCHER_STRING  = "LAUNCHER";
    public static final String ROTARY_STRING = "ROTARY";
    public static final DcMotorSimple.Direction ROTARY_DIRECTION = DcMotorSimple.Direction.REVERSE;


    public static int PURPLE_ARTIFACT_UPPER_HUE = 350;
    public static int PURPLE_ARTIFACT_LOWER_HUE = 275;

    public static int GREEN_ARTIFACT_UPPER_HUE = 150;
    public static int GREEN_ARTIFACT_LOWER_HUE = 100;

    public static final int SPACE_BETWEEN_ROTARY_SLOTS = 300;


    public static final double ANGLE_OF_LAUNCHER_IN_DEGREES = 35;
    public static  final double HEIGHT_OF_ROBOT_IN_METERS = 0.35;
    public static  final double HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS = (1.11125) - (HEIGHT_OF_ROBOT_IN_METERS);
    public static  final double MOTOR_TO_TURRET_RATIO = (double) 24 /155; //Motor to Turret
    public static  final double TURRET_TICKS_PER_DEGREE = 537.7/360;

    public static final String BALL_SERVO_STRING = "BS";

    public static final String TURRET_ROTATION_STRING = "TURR";




}
