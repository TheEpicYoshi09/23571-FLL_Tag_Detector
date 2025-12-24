package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class CRServoPositionControl {

    // general constants
    public static double maxVoltage = 3.26;
    public static double degreesPerRev = 360.0;

    // gains
    public static double kP = 0.002;
    public static double kI = 0.00000;
    public static double kD = 0.000;//idk if this works
    public static double kS = 0.0; // static friction feedforward
    public static double stiffnessGain = 1.0;

    // shaping
    public static double maxPower = 1.0;
    public static double brakeZoneDeg = 35.0;
    public static double brakeMaxPower = 0.85;// tighter clamp near target
    public static double deadbandDeg = 1.5;

    //safe d (kinda sounds like safe-ty
    public static double velFilterAlpha = 0.20;//lower is smooter
    public static double dTermClamp = 0.20;//caps braekig d

    // integral safety
    public static double integralLimit = 300.0;// clamp on integral accumulator

    // output slew limit powerunits per second
    public static double maxOutputSlewPerSec = 3.0;

    // stall
    public static double stallBoostPower = 1.0;
    public static double stallVelThresh = 50.0; //degrees perseconds
    public static double stallErrThresh = 20.0;  // deg

    // direction
    public static boolean rotateClockwise = true;

    private final CRServo servo;
    private final AnalogInput encoder;

    private double lastWrappedDeg;
    private double continuousDeg;
    private double targetDeg;

    // damping helpers
    private double lastAngleDeg = 0.0;
    private long lastTimeNs = 0;

    // new state
    private double velEma = 0.0;
    private double integral = 0.0;
    private double lastOutput = 0.0;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;

        double initial = getWrappedAngle();
        lastWrappedDeg = initial;
        continuousDeg = initial;
        targetDeg = initial;

        lastAngleDeg = continuousDeg;
        lastTimeNs = System.nanoTime();
    }

    public void update() {
        updateContinuousAngle();

        double error = targetDeg - continuousDeg;
        double absErr = Math.abs(error);

        // sigma sigma noy
        if (absErr < deadbandDeg) {
            servo.setPower(0);
            lastAngleDeg = continuousDeg;
            lastTimeNs = System.nanoTime();
            integral = 0.0;
            lastOutput = 0.0;
            return;
        }

        // velocity measurement
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        if (dt <= 0) dt = 1e-3;

        // velocity + filter
        double rawVel = (continuousDeg - lastAngleDeg) / dt; // deg/s
        velEma = velFilterAlpha * rawVel + (1 - velFilterAlpha) * velEma;

        lastAngleDeg = continuousDeg;
        lastTimeNs = now;

        //P shaping near target
        double brakeScale = (absErr < brakeZoneDeg) ? (0.4 + 0.6 * absErr / brakeZoneDeg) : 1.0;
        double kP_eff = kP * stiffnessGain * brakeScale;

        //Integral gating (only when near-ish and not moving too fast)
        if (absErr < 40.0 && Math.abs(velEma) < 150.0) {
            integral += error * dt;
            integral = clamp(integral, -integralLimit, integralLimit);
        } else {
            integral *= 0.9; // bleed
        }

        //D limited
        boolean movingTowardTarget = Math.signum(velEma) == Math.signum(error);
        double kD_eff = (absErr < brakeZoneDeg && movingTowardTarget) ? kD : 0.0;

        double pTerm = kP_eff * error;
        double iTerm = kI * integral;
        double dTerm = -kD_eff * velEma;
        dTerm = clamp(dTerm, -dTermClamp, dTermClamp);

        double output = pTerm + iTerm + dTerm;

        //sf compensation and erro
        double dir = Math.signum(output);
        if (dir == 0) dir = Math.signum(error);
        double kS_eff = kS * clamp(absErr / brakeZoneDeg, 0.3, 1.0);
        if (dir != 0 && Math.signum(error) == dir) {
            output += dir * kS_eff;
        }

        //clamp logic with stall assist
        double maxClamp = (absErr < brakeZoneDeg) ? brakeMaxPower : maxPower;
        if (Math.abs(velEma) < stallVelThresh && absErr > stallErrThresh) {
            maxClamp = Math.max(maxClamp, stallBoostPower);
        }

        //slew limit
        double maxDelta = maxOutputSlewPerSec * dt;
        output = clamp(output, lastOutput - maxDelta, lastOutput + maxDelta);

        //final clamp
        output = clamp(output, -maxClamp, maxClamp);
        lastOutput = output;

        servo.setPower(output);
    }

    public void moveToAngle(double wrappedAngleDeg) {
        updateContinuousAngle();

        double currentWrapped = mod(continuousDeg, 360.0);
        double delta = wrappedAngleDeg - currentWrapped;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        if (rotateClockwise && delta < 0) delta += 360;
        if (!rotateClockwise && delta > 0) delta -= 360;

        targetDeg = continuousDeg + delta;
    }

    public void moveBy(double deltaDeg) {
        targetDeg += deltaDeg;
    }

    public void reset() {
        double wrapped = getWrappedAngle();
        lastWrappedDeg = wrapped;
        continuousDeg = wrapped;
        targetDeg = wrapped;
        servo.setPower(0);
        integral = 0.0;
        lastOutput = 0.0;
    }

    private void updateContinuousAngle() {
        double wrapped = getWrappedAngle();
        double delta = wrapped - lastWrappedDeg;

        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousDeg += delta;
        lastWrappedDeg = wrapped;
    }

    private double getWrappedAngle() {
        double v = clamp(encoder.getVoltage(), 0.0, maxVoltage);
        return (v / maxVoltage) * degreesPerRev;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double mod(double v, double m) {
        double r = v % m;
        return (r < 0) ? r + m : r;
    }

    public double getCurrentAngle() { return continuousDeg; }
    public double getTargetAngle() { return targetDeg; }

    public double getTargetVoltage() {
        double wrappedDeg = targetDeg % degreesPerRev;
        if (wrappedDeg < 0) wrappedDeg += degreesPerRev;
        return (wrappedDeg / degreesPerRev) * maxVoltage;
    }

    public double getVoltage() { return encoder.getVoltage(); }
}