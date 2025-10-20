package org.firstinspires.ftc.teamcode.interstellar.directives;

import org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper.StellarServo;
import org.firstinspires.ftc.teamcode.interstellar.Subsystem;

public class SetPosition extends Directive {
	private final StellarServo servo;
	private final double targetPosition;

	public SetPosition(StellarServo servo, double targetPosition) {
		this.servo = servo;
		this.targetPosition = targetPosition;
	}

	@Override
	public void start(boolean interrupted) {
		servo.setPosition(targetPosition);
	}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {}

	public SetPosition requires(Subsystem... subsystems) {
		setRequires(subsystems);
		return this;
	}

	public SetPosition interruptible(boolean interruptible) {
		setInterruptible(interruptible);
		return this;
	}

	@Override
	public boolean isFinished() {
		return Math.abs(servo.getPosition() - targetPosition) <= 0.01;
	}
}
