package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ShooterClass;

@TeleOp(name = "ShooterClassTest", group = "Test")
public class ShooterClassTest extends LinearOpMode {

    private ShooterClass shooter;

    // Debounce flags
    private boolean previousLeftBumper = false;

    // Toggle states
    private boolean shooterHingeUp = false;

    @Override
    public void runOpMode() {

        // Initialize ShooterClass: shooter motor = true, hinge = true, magazine1 = "crservo", magazine2 = "none"
        shooter = new ShooterClass(hardwareMap, telemetry, true, true, "crservo", "none");

        telemetry.addData("Status", "Init complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Shooter Motor & Magazine --------------------
            shooter.shooterEnabled = gamepad2.right_trigger > 0.1;
            if (shooter.shooterEnabled) {
                shooter.shooterTargetRPM = 1200;   // example target RPM
                shooter.magazineTargetPower = 1.0;
                shooter.magazineEnabled = true;
            } else {
                shooter.magazineEnabled = false;
            }

            // -------------------- Hinge Toggle --------------------
            boolean currentLeftBumper = gamepad2.left_bumper;
            if (currentLeftBumper && !previousLeftBumper) {
                shooterHingeUp = !shooterHingeUp;
            }
            previousLeftBumper = currentLeftBumper;

            shooter.hingeEnabled = true;
            shooter.hingeTargetPosition = shooterHingeUp ? 1.0 : 0.0;

            // -------------------- Update Shooter --------------------
            shooter.update(shooter.shooterEnabled, shooter.hingeEnabled, shooter.magazineEnabled);

            // -------------------- Telemetry --------------------
            telemetry.addData("Shooter RPM", shooter.getShooterCurrentRPM());
            telemetry.addData("Shooter Target RPM", shooter.shooterTargetRPM);
            telemetry.addData("Shooter Hinge", shooterHingeUp ? "Up" : "Down");
            telemetry.addData("Magazine", shooter.magazineEnabled ? "On" : "Off");
            telemetry.update();
        }
    }
}
