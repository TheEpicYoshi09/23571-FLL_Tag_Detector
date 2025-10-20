package org.firstinspires.ftc.teamcode.interstellar.directives;

public class Sleep extends Directive {
	private final double sleepTimeSeconds;
	private long endingTime;

	public Sleep(double sleepTimeSeconds) {

		this.sleepTimeSeconds = sleepTimeSeconds;

		setInterruptible(true);
	}

	@Override
	public void start(boolean interrupted) {
		endingTime = System.currentTimeMillis() + (long) (sleepTimeSeconds * 1000.0);
	}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return System.currentTimeMillis() >= endingTime;
	}
}