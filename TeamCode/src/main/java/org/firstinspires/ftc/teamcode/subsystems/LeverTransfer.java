package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.interstellar.directives.SetPosition;
import org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper.StellarServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.interstellar.Subsystem;

public final class LeverTransfer extends Subsystem {
	private Gamepad gamepad1, gamepad2;
	private StellarServo leverTransfer;
	private final double leverDownPosition, leverUpPosition;
	private boolean leverTargetIsUpPosition = false;
	private ButtonMap dpadUpButtonMap, dpadDownButtonMap, dpadLeftButtonMap;

    public LeverTransfer(double leverDownPosition, double leverUpPosition) {
		this.leverDownPosition = leverDownPosition;
		this.leverUpPosition = leverUpPosition;
	}

	@Override
	public void init(HardwareMap hardwareMap) {
		leverTransfer = new StellarServo(hardwareMap, "leverTransfer");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;

		dpadUpButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_UP);
		dpadDownButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_DOWN);
		dpadLeftButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_LEFT);
	}

	@Override
	public void update() {
		//todo: stop command spam
		dpadUpButtonMap.handle(() -> {
			leverTargetIsUpPosition = true;
		});

		dpadDownButtonMap.handle(() -> {
			leverTargetIsUpPosition = false;
		});

		dpadLeftButtonMap.handle(() -> {
			leverTargetIsUpPosition = !leverTargetIsUpPosition;
		});

		new SetPosition(leverTransfer,
				leverTargetIsUpPosition ? leverUpPosition : leverDownPosition
		).interruptible(true).requires(this).schedule();

		//leverTransfer.setPosition(leverTargetIsUpPosition ? leverUpPosition : leverDownPosition);
	}

	@Override
	public String getTelemetryData() {
		return String.format("Lever Up Position: %f\nLever Is Up: %b", leverUpPosition, leverTargetIsUpPosition);
	}
}