package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class CRServoPositionControl {

    //CONFIGURABLES

    // general constants
    public static double maxVoltage = 3.26;
    public static double degreesPerRev = 360.0;

    // gains (insert muscle emoji here)
    public static double kP = 0.008;
    public static double kF = 0.06;
    public static double maxPower = 0.6;

    //deadbands
    public static double deadbandDeg = 1.5;
    public static double ffDeadbandDeg = 10.0;

    //direction
    public static boolean rotateClockwise = true;

    //STATE

    private final CRServo servo;
    private final AnalogInput encoder;

    // super sigma wrapping
    private double lastWrappedDeg = 0;
    private double continuousDeg = 0;

    // Target
    private double targetDeg = 0;

    /* ================= CONSTRUCTOR ================= */

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;

        double initial = getWrappedAngle();
        lastWrappedDeg = initial;
        continuousDeg = initial;
        targetDeg = continuousDeg;
    }

    //api
    //lwkey bruno mars and rose should make APT again but make it API instead would that be tuffy
    // something idk if we'll use but might have to do some refactorign if we do basically moves from current position
    private void moveDegrees(double deltaDegrees) {
        targetDeg += deltaDegrees;
    }

    // something idk we migh use this at somep point
    private void setTargetDegrees(double absoluteDegrees) {
        targetDeg = absoluteDegrees;
    }

    // dashign through the snow
    // in a one horse open sleigh
    // over the fields we go
    // laughing all the way
    // bells on bobtail ring
    // making spirits bright
    // what fun it is to ride and sing
    // a sleighing song tonight oh
    public void update() {
        updateContinuousAngle();

        double error = targetDeg - continuousDeg;

        // CW only stop condition
        if (error <= deadbandDeg) {
            servo.setPower(0);
            return;
        }

        double ff = (error > ffDeadbandDeg) ? kF : kF * (error / 10.0);
        double output = kP * error + ff;
        output = clamp(output, 0, maxPower); // CW-only power

        servo.setPower(output);
    }

    //bencoder (ben like a reference to ben falk who is majestic)
    private void updateContinuousAngle() {
        double wrapped = getWrappedAngle();
        double delta = wrapped - lastWrappedDeg;

        // unwrap
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousDeg += delta;
        lastWrappedDeg = wrapped;
    }

    private double getWrappedAngle() {
        double v = Math.max(0.0, Math.min(encoder.getVoltage(), maxVoltage));
        return (v / maxVoltage) * degreesPerRev;
    }

    /* ================= UTIL ================= */

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= DEBUG ================= */

    public double getTargetVoltage() {
        double wrappedDeg = targetDeg % degreesPerRev;
        if (wrappedDeg < 0) wrappedDeg += degreesPerRev;

        return (wrappedDeg / degreesPerRev) * maxVoltage;
    }

    public void moveToAngle(double wrappedAngleDeg) {
        updateContinuousAngle();

        double currentWrapped = continuousDeg % 360.0;
        if (currentWrapped < 0) currentWrapped += 360.0;

        double delta = wrappedAngleDeg - currentWrapped;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        if (rotateClockwise && delta < 0) delta += 360;
        if (!rotateClockwise && delta > 0) delta -= 360;

        targetDeg = continuousDeg + delta;
    }


    public double getCurrentContinuousAngle() {
        return continuousDeg;
    }

    public double getTargetAngle() {
        return targetDeg;
    }

    public void reset(Telemetry telem) {
        telem.addLine("CRServoPositionController reset");

        double wrapped = getWrappedAngle();
        lastWrappedDeg = wrapped;
        continuousDeg = wrapped;
        targetDeg = wrapped;
        servo.setPower(0);
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