package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSettings;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "Teleop")
public class RunTeleop extends OpMode {

    final private ElapsedTime teleopTimer = new ElapsedTime();
    final private ElapsedTime blinkTimer = new ElapsedTime();
    boolean endGameWarning = false;
    boolean lift_press = false;

    @Override
    public void init() {
        telemetry.addData(">", "Initializing hardware.");
        telemetry.update();
        AutoSettings.INSTANCE.readAutoConfig();
        Drive.INSTANCE.init(hardwareMap);
        Vision.INSTANCE.init(hardwareMap);
        Vision.INSTANCE.setAlliance(AutoSettings.INSTANCE.iAmBlue());
        Shooter.INSTANCE.init(hardwareMap);
        Lift.INSTANCE.init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        PinPoint.INSTANCE.init(hardwareMap);
        Odometry.INSTANCE.teleinit();
        telemetry.addData(">", "Initialization complete.");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        teleopTimer.reset();
        endGameWarning = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (teleopTimer.seconds() >= 80. & teleopTimer.seconds() <= 90.) {
            if (! endGameWarning) {
                blinkTimer.reset();
                endGameWarning = ! endGameWarning;
            }
            if (Math.round(blinkTimer.seconds()) % 2 == 1) {
                gamepad1.rumble(250);
            }
        }

       // boolean intakeInButton = gamepad1.left_trigger > 0.2;
        //boolean intakeOutButton = gamepad1.left_bumper;
        boolean intakeInButton = gamepad1.a;
        boolean intakeOutButton = gamepad1.b;
        if (intakeOutButton && intakeInButton) {
            intakeInButton = false;
            intakeOutButton = false;
        }

        boolean kickerBackButton = gamepad1.right_bumper;
        boolean kickerOnButton = gamepad1.right_trigger > 0.3;
        if (kickerOnButton && kickerBackButton) {
            kickerOnButton = false;
        }



        boolean shooterButtonHigh =  gamepad1.dpad_up;
        boolean shooterButtonMedium = gamepad1.dpad_right;
        boolean shooterButtonLow = gamepad1.dpad_down;
        boolean shooterButtonOff = gamepad1.dpad_left;

        // INTAKE CODE
        if (intakeInButton) {
            Intake.INSTANCE.intakein();
            telemetry.addLine("Intake: In");
        } else if (intakeOutButton) {
            Intake.INSTANCE.intakeout();
            telemetry.addLine("Intake: Out");
        } else {
            Intake.INSTANCE.intakeoff();
            telemetry.addLine("Intake: Off");
        }

        if (shooterButtonHigh) {
            Shooter.INSTANCE.high();
            telemetry.addLine("Shooter: ShootHigh");
        }
        if (shooterButtonMedium) {
            Shooter.INSTANCE.medium();
            telemetry.addLine("Shooter: shootMid");
        }
        if (shooterButtonLow) {
            Shooter.INSTANCE.high();
            telemetry.addLine("Shooter: ShootLow");
        }
        if (shooterButtonOff) {
            Shooter.INSTANCE.stop();
            telemetry.addLine("Shooter: Stop");
        }

        // LIFT CODE
        if (kickerOnButton) {
            Shooter.INSTANCE.kickeron();
            telemetry.addLine("Lift: Tip");
        } else if (kickerBackButton) {
            Shooter.INSTANCE.kickerout();
            telemetry.addLine("Lift: Stow");
        } else {
            Shooter.INSTANCE.kickeroff();
            telemetry.addLine("Lift: Off");
        }
        telemetry.addData("Lift: position:",Lift.INSTANCE.getPosition());

        double drive = -1. * squareInput(gamepad1.left_stick_y);
        double strafe = -1. * squareInput(gamepad1.left_stick_x);
        double turn = -1. * squareInput(gamepad1.right_stick_x);
        Drive.INSTANCE.moveRobot(drive, strafe, turn);

        telemetry.addData("Drive: ","powers: %5.2f / %5.2f / %5.2f",drive,strafe,turn);
        telemetry.update();
    }

    public double squareInput(double stick) {
        return stick*stick*stick;
    }
}
