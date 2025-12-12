package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class CRServoPositionControl
{
    private final CRServo crServo;
    private final AnalogInput encoder;
    private ElapsedTime timer = new ElapsedTime();

    // Gains
    public static double kp = 0.41;
    public static double ki = 0.0;
    public static double kf = 0.01;
    public static double filterAlpha = 0.9;

    // dncoder constants
    public static double ticksPerRev = 3.3;
    public static double degreesPerRev = 360.0;

    // Offset & deadband
    public static double constantOffset = 0.15;
    public static double deadbandAngles = 1.67;
    private double deadband = deadbandAngles / degreesPerRev * ticksPerRev;

    // state
    private double integral = 0;
    private double targetVoltage_actual = 0;
    private double targetVoltage_offset = 0;

    // filtering type shift
    private double lastRawVoltage = 0;
    private double unwrappedVoltage = 0;
    private double filteredVoltage = 0;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder)
    {
        this.crServo = servo;
        this.encoder = encoder;
    }

    // 2+2 is 4 minus one thats three quick maths
    public void moveToAngle(double targetAngleDegrees)
    {
        // Real target voltage
        targetVoltage_actual = angleToVoltage(targetAngleDegrees);

        // Apply offset in UNWRAPPED domain later
        targetVoltage_offset = wrapVoltage(targetVoltage_actual + constantOffset);

        // Current
        double measuredContinuous = getFilteredVoltage();  // continuous
        double measuredOffset = wrapVoltage(measuredContinuous + constantOffset);

        // shortest error
        double error = shortestError(targetVoltage_offset, measuredOffset);

        // Deadband
        if (Math.abs(error) <= deadband) {
            crServo.setPower(0);
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        timer.reset();

        // integral += error * dt;
        // integral = Math.max(-2, Math.min(2, integral));

        double output = kp * error /*+ ki * integral*/ + kf * Math.signum(error);
        output = Math.max(-1, Math.min(1, output));

        crServo.setPower(output);
    }

    // rudolph the red nosed reindeer
    // had a very shiny nose
    // and if you ever saw him
    // you would even say it glows
    // all of the other reindeer
    // used to laugh and call him names
    // they never let poor rudolph
    // join in any reindeer games
    // then one foggy christmas eve
    // santa came to say
    // "rudolph with your nose so bright
    // won't you guide my sleigh tonight"
    // oh then all the reindeer loved him
    // as they shouted out with glee
    // "rudolph the red nosed reindeer
    // you'll go down in history"
    private double velocity = 0;

    private double getFilteredVoltage()
    {
        double raw = encoder.getVoltage();

        double diff = raw - filteredVoltage;
        if (diff >  ticksPerRev/2) diff -= ticksPerRev;
        if (diff < -ticksPerRev/2) diff += ticksPerRev;

        velocity = (1 - filterAlpha) * velocity + filterAlpha * diff;

        filteredVoltage += velocity;
        return filteredVoltage;
    }


    // utils and such
    private double wrapVoltage(double v)
    {
        double wrapped = v % ticksPerRev;
        if (wrapped < 0) wrapped += ticksPerRev;
        return wrapped;
    }

    private double shortestError(double target, double current)
    {
        // Inputs are wrapped 0–ticksPerRev
        double diff = target - current;

        if (diff >  ticksPerRev / 2) diff -= ticksPerRev;
        if (diff < -ticksPerRev / 2) diff += ticksPerRev;

        return diff;
    }

    private double angleToVoltage(double angleDegrees)
    {
        double a = Math.max(0, Math.min(degreesPerRev, angleDegrees));
        return (a / degreesPerRev) * ticksPerRev;
    }

    public void reset(Telemetry telem)
    {
        telem.addData("CRServoPositionControl", "Resetting controller");
        timer = new ElapsedTime();

        integral = 0;
        targetVoltage_actual = 0;
        targetVoltage_offset = 0;

        lastRawVoltage = encoder.getVoltage();
        filteredVoltage = lastRawVoltage;
        velocity = 0;
        deadband = deadbandAngles / degreesPerRev * ticksPerRev;
    }


    // getters for sigma debug
    public double getActualTargetVoltage() { return targetVoltage_actual; }
    public double getOffsetTargetVoltage() { return targetVoltage_offset; }

    public double getOffsetMeasuredVoltage()
    {
        return wrapVoltage(getFilteredVoltage() + constantOffset);
    }

    public double getCurrentAngle()
    {
        double wrapped = wrapVoltage(getFilteredVoltage());
        return (wrapped / ticksPerRev) * degreesPerRev;
    }
}

/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder;

    private static boolean angleIsLocked = false;
    private static double lockedAngle;
    public static double kp = 0.341;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0167;
    public static double lockedkp = 0.341;
    public static double lockedki = 0.0;
    public static double lockedkd = 0.0;
    public static double lockedkf = 0.0167;
    public static double filterAlpha = 0.80;
    public static double angleDeadband = 1.67;


    // Dynamic speed parameters
    public static double minSpeed = 0.15;           // minimal power to overcome deadband
    public static double maxErrorForScaling = 90.0; // error threshold for full speed

    private double integral = 0.0;
    private double lastError = 0.0;
    private Double filteredVoltage = null;
    public final double MAX_VOLTAGE;
    private ElapsedTime timer = new ElapsedTime();
    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        this.MAX_VOLTAGE = encoder.getMaxVoltage();
        timer.reset();
    }

    private double getFilteredVoltage() {
        if (filteredVoltage == null) {
            filteredVoltage = encoder.getVoltage();  // initialize to first reading
        } else {
            filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        }
        return filteredVoltage;
    }

    double getAngle() {
        return (getFilteredVoltage() / MAX_VOLTAGE) * 360.0;
    }

    public void lockAngle(double targetAngleDegrees) {
        angleIsLocked = true;
        lockedAngle = targetAngleDegrees;
    }
    public void unlockAngle() {
        angleIsLocked = false;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double tempkp = kp;
        double tempki = ki;
        double tempkd = kd;
        double tempkf = kf;
        if (angleIsLocked) {
            targetAngleDegrees = lockedAngle;
            tempkp = lockedkp;
            tempki = lockedki;
            tempkd = lockedkd;
            tempkf = lockedkf;
        }

        double currentAngle = getAngle();
        double error = ((targetAngleDegrees - currentAngle + 540) % 360) - 180;

        if (Math.abs(error) < angleDeadband) {
            crServo.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));

        double derivative = (error - lastError) / deltaTime;

        double output = tempkp * error + tempki * integral + tempkd * derivative + tempkf * Math.signum(error);

        // Dynamic speed scaling, speeds up if distance is further
        double distanceFactor = Math.min(Math.abs(error) / maxErrorForScaling, 1.0);
        double scaledPower = minSpeed + (1.0 - minSpeed) * distanceFactor;
        output = Math.signum(output) * Math.min(Math.abs(output), scaledPower);

        crServo.setPower(output);
        lastError = error;
    }
}
*/