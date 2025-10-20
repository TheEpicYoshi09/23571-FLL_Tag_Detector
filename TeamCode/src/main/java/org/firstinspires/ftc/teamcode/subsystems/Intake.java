package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.interstellar.directives.SetPower;
import org.firstinspires.ftc.teamcode.interstellar.hardwaremapwrapper.StellarDcMotor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.interstellar.Subsystem;


public final class Intake extends Subsystem {
	private Gamepad gamepad1, gamepad2;
	private StellarDcMotor intake;
	private double intakeSpeed = 0;

	@Override
	public void init(HardwareMap hardwareMap) {
		intake = new StellarDcMotor(hardwareMap, "intake");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void update() {
		//todo: currently floods too many directives
		if (gamepad1.right_trigger > 0.05) {
			intakeSpeed = gamepad1.right_trigger;
		} else if (gamepad1.left_trigger > 0.05) {
			intakeSpeed = -gamepad1.left_trigger;
		} else {
			intakeSpeed = 0;
		}

		new SetPower(intake, intakeSpeed).requires(this).schedule();

		//intake.setPower(intakeSpeed);
	}

	@Override
	public String getTelemetryData() {
		return String.format("Intake Speed: %f", intakeSpeed);
	}
}