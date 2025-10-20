package org.firstinspires.ftc.teamcode.interstellar.directives;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.interstellar.actions.Action;
import org.firstinspires.ftc.teamcode.interstellar.actions.BooleanAction;
import org.firstinspires.ftc.teamcode.interstellar.Subsystem;

import java.util.function.BooleanSupplier;

public class LambdaDirective extends Directive {
	private BooleanAction onStart = (interrupted) -> {};
	private Action onUpdate = () -> {};
	private BooleanAction onStop = (interrupted) -> {};
	private BooleanSupplier finishedWhen = () -> true;

	public LambdaDirective() {}

	public LambdaDirective onStart(BooleanAction onStart) {
		this.onStart = onStart;
		return this;
	}

	@Override
	public void start(boolean interrupted) {
		onStart.run(interrupted);
	}

	public LambdaDirective onUpdate(Action onUpdate) {
		this.onUpdate = onUpdate;
		return this;
	}

	@Override
	public void update() {
		onUpdate.run();
	}

	public LambdaDirective onStop(BooleanAction onStop) {
		this.onStop = onStop;
		return this;
	}

	@Override
	public void stop(boolean interrupted) {
		onStop.run(interrupted);
	}

	public LambdaDirective finishedWhen(BooleanSupplier finishedWhen) {
		this.finishedWhen = finishedWhen;
		return this;
	}

	@Override
	public boolean isFinished() {
		return finishedWhen.getAsBoolean();
	}

	public LambdaDirective requires(@NonNull Subsystem... subsystems) {
		setRequires(subsystems);
		return this;
	}

	public LambdaDirective interruptible(boolean isInterruptible) {
		setInterruptible(isInterruptible);
		return this;
	}
}