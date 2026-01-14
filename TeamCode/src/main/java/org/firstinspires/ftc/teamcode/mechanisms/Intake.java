package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for the robot's intake mechanism.
 * Controls the intake motor and the loading motor.
 */
public class Intake {

    private DcMotor intakeMotor, loadMotor;
    double timer = 0;

    /**
     * Initializes the intake hardware.
     * 
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        loadMotor = hwMap.get(DcMotor.class, "loadMotor");
    }

    /**
     * Sets the power of the intake motor.
     * 
     * @param intakeSpeed Power level (-1.0 to 1.0)
     */
    public void intake(double intakeSpeed) {
        intakeMotor.setPower(intakeSpeed);
    }

    /**
     * Controls the loading mechanism with a pulsed action.
     * WARNING: This method relies on loop count for timing, which is inconsistent.
     * Consider using System.currentTimeMillis() or ElapsedTime for better accuracy.
     * 
     * @param loadSpeed Power level for the loader
     * @param interval  Number of loops to run the loader
     * @param cooldown  Number of loops to pause the loader
     */
    public void load(double loadSpeed, double interval, double cooldown) {

        if (timer <= interval) {
            loadMotor.setPower(loadSpeed);
            timer += 1;
        } else if (timer <= interval + cooldown) {
            loadMotor.setPower(0);
            timer += 1;
        } else {
            timer = 0;
        }
    }

    /**
     * Stops all intake motors immediately.
     * Resets the loader timer state.
     */
    public void stopAll() {
        intakeMotor.setPower(0);
        loadMotor.setPower(0);
        timer = 0;
    }
}
