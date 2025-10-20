package org.firstinspires.ftc.teamcode.interstellar;

import org.firstinspires.ftc.teamcode.interstellar.directives.Directive;
import java.util.ArrayList;
import java.util.List;

public class Scheduler {
	private static final Scheduler instance = new Scheduler();

	private Scheduler() {}

	public static Scheduler getInstance() {
		return instance;
	}

	private final List<Directive> runningDirectives = new ArrayList<>();

	public void schedule(Directive directive) {
		//todo: add subsystems req logic
		runningDirectives.add(directive);
		directive.start(false);
	}

	public void run() {
		//todo: add sequences and interrupting
		//todo: be able to add subsystems other than `this` to requirements
		for (Directive directive : runningDirectives) {
			if (directive.isFinished()) {
				directive.stop(false);
				runningDirectives.remove(directive);
			} else {
				directive.update();
			}
		}
	}

	public void cancelAll() {
		for (Directive directive : runningDirectives) {
			directive.stop(true);
		}
		runningDirectives.clear();
	}
}
