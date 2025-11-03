package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

// [mike] I could be wrong about this, but since Directive extends Runnable, it might
// already expose these 4 functions, as well as Runnable's other public functions.
// That might be fine though -- does Directive have a special meaning beyond what
// Runnable does?
public abstract class Directive extends Runnable {
	public abstract void start(boolean hadToInterruptToStart);

	public abstract void update();

	public abstract void stop(boolean interrupted);

	public abstract boolean isFinished();
}
