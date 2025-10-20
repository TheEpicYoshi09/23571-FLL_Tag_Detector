package org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StellarDcMotor {
	private final DcMotor dcMotor;
	public StellarDcMotor(HardwareMap hardwareMap, String dcMotorName) {dcMotor = hardwareMap.get(DcMotor.class, dcMotorName);}
	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {dcMotor.setZeroPowerBehavior(zeroPowerBehavior);}
	public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {return dcMotor.getZeroPowerBehavior();}
	public void setTargetPosition(int targetPosition) {dcMotor.setTargetPosition(targetPosition);}
	public int getTargetPosition() {return dcMotor.getTargetPosition();}
	public void setPower(double power) {dcMotor.setPower(power);}
	public double getPower() {return dcMotor.getPower();}
	public void setMode(DcMotor.RunMode mode) {dcMotor.setMode(mode);}
	public DcMotor.RunMode getMode() {return dcMotor.getMode();}
	public void setDirection(DcMotor.Direction direction) {dcMotor.setDirection(direction);}
	public DcMotor.Direction getDirection() {return dcMotor.getDirection();}
	public int getCurrentPosition() {return dcMotor.getCurrentPosition();}
	public boolean isBusy() {return dcMotor.isBusy();}
	public void resetDeviceConfigurationForOpMode() {dcMotor.resetDeviceConfigurationForOpMode();}
}