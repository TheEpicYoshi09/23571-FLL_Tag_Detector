package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.stellarstructure.StellarBot;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class TarsBase extends StellarBot {
	public TarsBase() {
		super(
				Drivebase.getInstance(),
				Intake.getInstance(),
				LeverTransfer.getInstance(),
				Spindexer.getInstance()
		);
	}
}