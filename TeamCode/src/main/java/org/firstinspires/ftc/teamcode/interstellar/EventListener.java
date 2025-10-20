package org.firstinspires.ftc.teamcode.interstellar;

import org.firstinspires.ftc.teamcode.interstellar.directives.Directive;

import java.util.function.BooleanSupplier;

public class EventListener {
	private final BooleanSupplier condition;
	private final Directive directive;

	public EventListener(BooleanSupplier condition, Directive directive) {
		this.condition = condition;
		this.directive = directive;
	}

	public void update() {
		if (condition.getAsBoolean()) {
			directive.schedule();
		}
	}
}
