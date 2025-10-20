package org.firstinspires.ftc.teamcode.interstellar.directives;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper.StellarDcMotor;
import org.firstinspires.ftc.teamcode.interstellar.Subsystem;

public class MoveTo extends Directive {
	private final StellarDcMotor motor;
	private final int targetPosition;
	private final double power;

	public MoveTo(StellarDcMotor motor, int targetPosition, double power) {
		this.motor = motor;
		this.targetPosition = targetPosition;
		this.power = power;
	}

	@Override
	public void start(boolean interrupted) {
		motor.setTargetPosition(targetPosition);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(Math.abs(power));
	}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {
		motor.setPower(0);
	}

	public MoveTo requires(Subsystem... subsystems) {
		setRequires(subsystems);
		return this;
	}

	public MoveTo interruptible(boolean interruptible) {
		setInterruptible(interruptible);
		return this;
	}

	@Override
	public boolean isFinished() {
		return Math.abs(motor.getCurrentPosition() - targetPosition) <= 5;
	}
}
