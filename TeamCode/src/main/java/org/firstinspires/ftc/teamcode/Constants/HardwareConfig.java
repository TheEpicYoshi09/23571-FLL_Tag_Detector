package org.firstinspires.ftc.teamcode.Constants;

/**
 * Static configuration class that holds all hardware map names in one place.
 * This centralizes all hardware device names used in HardwareMap.get() calls.
 */
public class HardwareConfig {
    
    // Private constructor to prevent instantiation
    private HardwareConfig() {
        throw new IllegalStateException("Utility class");
    }
    
    // ========== DRIVE TRAIN MOTORS ==========
    public static final String LEFT_FRONT_MOTOR = "leftfrontmotor";
    public static final String RIGHT_FRONT_MOTOR = "rightfrontmotor";
    public static final String LEFT_BACK_MOTOR = "leftbackmotor";
    public static final String RIGHT_BACK_MOTOR = "rightbackmotor";
    
    // ========== FLYWHEEL MOTORS ==========
    public static final String FLYWHEEL_MOTOR = "flywheelmotor";
    public static final String FLYWHEEL_MOTOR_2 = "flywheelmotor2";
    
    // ========== LIFT MOTORS ==========
    public static final String LEFT_VIPER = "leftviper";
    public static final String RIGHT_VIPER = "rightviper";
    
    // ========== SERVOS ==========
    public static final String HOOD_SERVO = "hoodservo";
    public static final String INDEX_SERVO = "indexServo";
    public static final String TURRET_SERVO = "turretServo";
    public static final String KICKER_SERVO = "kicker";
    public static final String PIE_1_SERVO = "Pie1Servo";
    public static final String BALL_LAUNCHER_SERVO = "balllauncherservo";
    public static final String BALL_LAUNCHER = "balllauncher";
    
    // ========== CONTINUOUS ROTATION SERVOS ==========
    public static final String INTAKE_SERVO = "intakeServo";
    public static final String KICKER_WHEEL = "kickerwheel";
    
    // ========== SENSORS ==========
    public static final String LIMELIGHT = "limelight";
    public static final String INTAKE_COLOR_SENSOR = "intakeSensor";
    public static final String SHOOTER_COLOR_SENSOR = "shooterSensor";
    public static final String IMU = "imu";

}
