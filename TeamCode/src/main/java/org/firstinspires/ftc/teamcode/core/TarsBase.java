package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.interstellar.InterstellarBot;
import org.firstinspires.ftc.teamcode.interstellar.directives.LambdaDirective;
import org.firstinspires.ftc.teamcode.interstellar.directives.Sleep;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class TarsBase extends InterstellarBot {
	private final Drivebase drivebase;
	private final Intake intake;
	private final LeverTransfer leverTransfer;
	private final Spindexer spindexer;

	public TarsBase() {
		super(
				new Drivebase(1.00, 1.00),
				new Intake(),
				new LeverTransfer(0.28, 0.00),
				new Spindexer()
		);

		drivebase = (Drivebase) subsystems[0];
		intake = (Intake) subsystems[1];
		leverTransfer = (LeverTransfer) subsystems[2];
		spindexer = (Spindexer) subsystems[3];

		/*
		AtomicInteger number = new AtomicInteger(2);
		AtomicBoolean test = new AtomicBoolean();

		LambdaDirective directive = new LambdaDirective()
				.onStart((interrupted) -> {})
				.onUpdate(() -> number.set(number.get() + 1))
				.onStop(test::set)
 				.requires(drivebase, intake)
				.interruptible(true)
				.finishedWhen(() -> number.get() == 4);

		Sleep fiveSecondWait = new Sleep(3);
		 */
	}
}