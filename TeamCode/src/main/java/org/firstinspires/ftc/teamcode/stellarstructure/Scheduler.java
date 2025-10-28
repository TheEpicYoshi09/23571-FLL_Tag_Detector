package org.firstinspires.ftc.teamcode.stellarstructure;

import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Runnable;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Scheduler {
	private static final Scheduler instance = new Scheduler();

	private Scheduler() {}

	public static Scheduler getInstance() {
		return instance;
	}

	private final List<Subsystem> subsystems = new ArrayList<>();
	private final List<Runnable> activeRunnables = new ArrayList<>();
	private final List<Trigger> activeTriggers = new ArrayList<>();

	public void addSubsystem(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public void schedule(Runnable runnableToSchedule) {
		// prevent scheduling of the same directive multiple times
		if (this.activeRunnables.contains(runnableToSchedule)) {
			return;
		}

		boolean didInterrupt = false;

		// for every running directive
		for (Iterator<Runnable> iterator = this.activeRunnables.iterator(); iterator.hasNext();) {
			Runnable activeRunnable = iterator.next();

			// check for conflicts
			CONFLICT_CHECK:
			// for every subsystem required by the new directive
			for (Subsystem requiredByNew : runnableToSchedule.getRequiredSubsystems()) {
				// for every subsystem required by the running directive
				for (Subsystem requiredByRunning : activeRunnable.getRequiredSubsystems()) {
					if (requiredByNew == requiredByRunning) {
						// CONFLICT FOUND!

						if (activeRunnable.isInterruptible()) {
							// if the running command is interruptible, stop it and remove it.
							activeRunnable.stop(true);
							iterator.remove();
							didInterrupt = true;
						} else {
							// running command unable to be interrupted, so can't schedule new directive
							return;
						}

						// checked requirements for this runningDirective, move to next
						break CONFLICT_CHECK;
					}
				}
			}
		}

		runnableToSchedule.setHasFinished(false); // in case it has finished earlier

		this.activeRunnables.add(runnableToSchedule); // add to running directives
		runnableToSchedule.start(didInterrupt); // start directive and pass hadToInterruptToStart status
	}

	public void addTrigger(Trigger trigger) {
		activeTriggers.add(trigger);
	}

	public void run() {
		// check and run all triggers
		for (Trigger trigger : activeTriggers) {
			if (trigger.check()) {
				trigger.run();
			}
		}

		// update directives and remove finished directives
		for (Iterator<Runnable> iterator = activeRunnables.iterator(); iterator.hasNext();) {
			Runnable runnable = iterator.next();
			if (runnable.isFinished()) {
				runnable.stop(false);
				runnable.setHasFinished(true);
				iterator.remove();
			} else {
				runnable.update();
			}
		}

		// if subsystem isn't being used, then schedule default directive
		for (Subsystem subsystem : subsystems) {
			Runnable defaultDirective = subsystem.getDefaultDirective();

			if (defaultDirective != null && !isSubsystemInUse(subsystem)) {
				schedule(defaultDirective);
			}
		}
	}

	private boolean isSubsystemInUse(Subsystem subsystemToCheck) {
		//for every running directive
		for (Runnable activeRunnable : activeRunnables) {
			//check if running directive requires subsystem
			//includes the default directive itself but that's fine for this application
			for (Subsystem requiredSubsystem : activeRunnable.getRequiredSubsystems()) {
				if (requiredSubsystem == subsystemToCheck) {
					return true;
				}
			}
		}

		return false;
	}

	public void cancelAll() {
		//clear all running triggers
		activeTriggers.clear();

		//stop all directives
		for (Runnable runnable : activeRunnables) {
			runnable.stop(true);
		}

		//clear all running directives
		activeRunnables.clear();
	}
}
