package org.firstinspires.ftc.teamcode.interstellar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


//Stellar, Wormholes, Docking Procedure, Murphy's Law, Procedure

public class InterstellarBot {
	protected final Subsystem[] subsystems;

	public InterstellarBot(Subsystem... subsystems) {
		this.subsystems = subsystems;
	}

	public void init(HardwareMap hardwareMap) {
		for (Subsystem subsystem : subsystems) {
			subsystem.init(hardwareMap);
		}
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		for (Subsystem subsystem : subsystems) {
			subsystem.setGamepads(gamepad1, gamepad2);
		}
	}

	public void update() {
		Scheduler.getInstance().run();
		/*
		final AtomicInteger test = new AtomicInteger(2);
		Condition condition = new Condition(() -> test.get() == 3);
		condition.updateState();
		//condition.getState() == false
		test.set(2);
		condition.updateState();
		//condition.getState() == true
		*/


		for (Subsystem subsystem : subsystems) {
			subsystem.update();
		}
	}

	public String getTelemetryData() {
		StringBuilder telemetry = new StringBuilder();
		for (Subsystem subsystem: subsystems) {
			telemetry.append(subsystem.getTelemetryData()).append('\n');
		}
		return telemetry.toString();
	}

	public void cancelAll() {
		Scheduler.getInstance().cancelAll();
	}
}