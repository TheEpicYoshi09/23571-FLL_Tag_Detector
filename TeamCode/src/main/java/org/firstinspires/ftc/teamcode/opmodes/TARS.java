package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.TarsBase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TARS -JetEngine", group = "Robot")
public class TARS extends LinearOpMode {
	final TarsBase tars = new TarsBase();

	@Override
	public void runOpMode() {
		// set up subsystems
		tars.init(hardwareMap);

		// set up gamepads
		tars.setGamepads(gamepad1, gamepad2);

		waitForStart();

		if (isStopRequested()) return;

		while (opModeIsActive()) {
			// run subsystems logic
			tars.update();

			// print telemetry
			try {
				telemetry.addLine(tars.getTelemetryData());
			} catch (Exception e) {
				telemetry.addData("telemetry didn't work", e);
			}
			telemetry.addLine("Honesty setting: 90%");
			telemetry.addLine("Humor setting: 75%");

			telemetry.update();
		}
		tars.cancelAll();
	}
}