package org.firstinspires.ftc.teamcode.interstellar;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.interstellar.actions.Action;

public class Trigger {
	//todo: implement with condition and action
	//note: not being used yet
	public enum TriggerType {
		WHILE_PRESSED,
		ON_INITIAL_PRESS,
		ON_RELEASE
	}

	public enum Button {
		A, B, X, Y,
		LEFT_BUMPER, RIGHT_BUMPER,
		DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT
	}

	private final Gamepad gamepad;
	private final TriggerType triggerType;
	private final Button button;
	private boolean previousState, currentState = false;

	public Trigger(Gamepad gamepad, TriggerType triggerType, Button button) {
		this.gamepad = gamepad;
		this.triggerType = triggerType;
		this.button = button;
	}

	public void updateState() {
		previousState = currentState;
		currentState = isPressed();
	}

	public boolean isPressed() {
		switch (button) {
			case A:
				return gamepad.a;
			case B:
				return gamepad.b;
			case X:
				return gamepad.x;
			case Y:
				return gamepad.y;
			case LEFT_BUMPER:
				return gamepad.left_bumper;
			case RIGHT_BUMPER:
				return gamepad.right_bumper;
			case DPAD_UP:
				return gamepad.dpad_up;
			case DPAD_DOWN:
				return gamepad.dpad_down;
			case DPAD_LEFT:
				return gamepad.dpad_left;
			case DPAD_RIGHT:
				return gamepad.dpad_right;
			default:
				return false;
		}
	}

	public boolean evaluate() {
		switch (triggerType) {
			case WHILE_PRESSED:
				return currentState;
			case ON_INITIAL_PRESS:
				return currentState && !previousState;
			case ON_RELEASE:
				return !currentState && previousState;
			default:
				return false;
		}
	}

	public boolean handle(Action action) {
		boolean isTriggered = evaluate();
		if (isTriggered) {
			action.run();
		}
		return isTriggered;
	}
}