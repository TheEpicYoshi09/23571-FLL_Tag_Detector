package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

public class VelocityProfiler {

    //double value = 0;



    double smoothingFactor;

    double currentValue1 = 0;
    double currentValue2 = 0;
    double currentValue3 = 0;
    double currentValue4 = 0;


    public VelocityProfiler(double smoothingFactor){
    this.smoothingFactor = Range.clip(smoothingFactor,0.0,1.0);
    }

    //ElapsedTime timer = new ElapsedTime();

    //double maxRate = 2.0; // maximum rate of change of power allowed per second


//    public void resetElapsedTime(){
//        timer.reset();
//    };

//    public double velocityProfileIncrement(double value){
//        if(timer.milliseconds() >= 200) {
//        this.value = value;
//        if (value - currentValue4 > 0){
//            currentValue4 = currentValue4 + 0.2;
//        } else if (value - currentValue4 == 0) {
//            currentValue4 = currentValue4 + 0;
//        } else if (value - currentValue4 < 0) {
//            currentValue4 = currentValue4 - 0.2;
//        }
//            resetElapsedTime();
//        }
//        return currentValue4;
//    }



//    public double update(double targetValue) { // targetvalue is the value u wan ur motors to reach
//
//        // Time since last update (seconds)
//        double deltaTime = timer.seconds();
//        timer.reset();
//
//        // Maximum change allowed this update
//        double maxChange = maxRate * deltaTime;
//
//        // Difference between target and current value
//        double error = targetValue - currentValue4;
//
//        // If we can reach the target this update, snap to it
//        if (Math.abs(error) <= maxChange) {
//            currentValue4 = targetValue;
//        }
//        // Otherwise, move toward the target at the allowed rate
//        else {
//            currentValue4 += Math.signum(error) * maxChange;
//        }
//
//        return currentValue4;
//    }

    public double velocityProfileInput1(double inputTarget){
        currentValue1 = currentValue1 + (inputTarget - currentValue1) * smoothingFactor;
        return currentValue1;
    }
    public double velocityProfileInput2(double inputTarget){
        currentValue2 = currentValue2 + (inputTarget - currentValue2) * smoothingFactor;
        return currentValue2;
    }
    public double velocityProfileInput3(double inputTarget){
        currentValue3 = currentValue3 + (inputTarget - currentValue3) * smoothingFactor;
        return currentValue3;
    }

    public double velocityProfileInput4(double inputTarget){
        currentValue4 = currentValue4 + (inputTarget - currentValue4) * smoothingFactor;
        return currentValue4;
    }
// double power = velocityProfileIncrement(gamepad1.left_stick_y)

}
