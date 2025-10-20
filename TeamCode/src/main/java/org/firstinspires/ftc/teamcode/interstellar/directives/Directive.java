package org.firstinspires.ftc.teamcode.interstellar.directives;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.interstellar.Scheduler;
import org.firstinspires.ftc.teamcode.interstellar.Subsystem;

public abstract class Directive {
	//no required subsystems by default
	private Subsystem[] requiredSubsystems = {};

	//not interruptible by default
	private boolean interruptible = false;

	public abstract void start(boolean interrupted);

	public abstract void update();

	public abstract void stop(boolean interrupted);

	public abstract boolean isFinished();

	public final void setRequires(@NonNull Subsystem... subsystems) {
		requiredSubsystems = subsystems;
	}

	public final Subsystem[] getRequiredSubsystems() {
		return requiredSubsystems;
	}

	public final void setInterruptible(boolean interruptible) {
		this.interruptible = interruptible;
	}

	public final boolean isInterruptible() {
		return interruptible;
	}

	public final void schedule() {
		Scheduler.getInstance().schedule(this);
	}
}
