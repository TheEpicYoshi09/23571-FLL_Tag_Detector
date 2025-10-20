package org.firstinspires.ftc.teamcode.interstellar.directives;

import java.util.function.BooleanSupplier;

public class WaitUntil extends Directive {
	private final BooleanSupplier condition;
	public WaitUntil(BooleanSupplier condition) {
		this.condition = condition;
		setInterruptible(true);
	}

	@Override
	public void start(boolean interrupted) {}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return condition.getAsBoolean();
	}
}