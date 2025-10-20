package org.firstinspires.ftc.teamcode.interstellar.directives;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.interstellar.Subsystem;
import org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper.StellarDcMotor;

public class SetPower extends Directive {
	private final StellarDcMotor motor;
	private final double power;

	public SetPower(StellarDcMotor motor, double power) {
		this.motor = motor;
		this.power = power;
	}

	@Override
	public void start(boolean interrupted) {
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setPower(power);
	}

	@Override
	public void update() {

	}

	@Override
	public void stop(boolean interrupted) {

	}

	public SetPower requires(Subsystem... subsystems) {
		setRequires(subsystems);
		return this;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}