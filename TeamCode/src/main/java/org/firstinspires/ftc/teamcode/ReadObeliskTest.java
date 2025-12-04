package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ReadObelisk;

/**
 * TeleOp helper to exercise the ReadObelisk helper in isolation. Drives the turret toward
 * the appropriate alliance sweep limit, reports progress over telemetry, and caches any
 * detected obelisk pattern for reuse in other modes.
 */
@TeleOp(name = "Read Obelisk Test", group = "Test")
public class ReadObeliskTest extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(this);
    private ReadObelisk obeliskReader = new ReadObelisk(robot, this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        if (robot.limelight == null || robot.turret == null) {
            telemetry.addLine("ERROR: Limelight and turret must be initialized for obelisk testing.");
            telemetry.update();
            return;
        }

        ReadObelisk.ObeliskPattern detectedPattern = obeliskReader.scanForPattern();

        boolean allianceColor = robot.refreshAllianceFromSwitchState();
        telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
        telemetry.addData("Turret Sweep", "Complete");
        telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
        telemetry.addData("Target Found", detectedPattern != null);
        if (detectedPattern != null) {
            telemetry.addData("Pattern", detectedPattern);
        }
        telemetry.update();
    }
}

