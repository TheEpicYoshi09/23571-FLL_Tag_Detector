package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class Bot {
    private final Intake intake;
    private final Indexer indexer;
    private final Actuator actuator;
    private final Outtake outtake;
    private final Movement movement;

    private final GamepadEx g1;
    private final GamepadEx g2;
    private final Telemetry telemetry;

    private boolean fieldCentric = false;

    public enum FSM{
        SortOuttake,
        QuickOuttake,
        Intake,
        Endgame
    }

    public FSM state;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap);
        movement = new Movement(hardwareMap);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        telemetry = tele;
        state = FSM.Intake;
    }
    public void teleopInit() {
        indexer.startIntake();
    }
    public void teleopTick() {
        g1.readButtons();
        g2.readButtons();

        if (fieldCentric) {
            movement.teleopTickFieldCentric(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    0,
                    true
            );
        } else {
            movement.teleopTick(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    0
            );
        }
        if (g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) fieldCentric = !fieldCentric;

        switch(state) {

            case Intake:
                if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01) {
                    intake.run();
                } else {
                    intake.stop();
                }
                if (g2.wasJustPressed(GamepadKeys.Button.A)){
                    state = FSM.QuickOuttake;
                }
                //TODO: manage spinning while intaking
                break;
            case QuickOuttake:
                if (g2.wasJustPressed(GamepadKeys.Button.X)){
                    //TODO: quickspin
                }
                if (g2.wasJustPressed(GamepadKeys.Button.A)){
                    state = FSM.Intake;
                }
                break;
            case SortOuttake:
                //TODO: figure this out
                break;
            case Endgame:
                //TODO: Ts more of a build todo than a code one
        }
        // ========== TELEMETRY ==========
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Indexer State", indexer.getState());
        telemetry.addData("Next State", indexer.nextState());
        telemetry.addData("Indexer Voltage", indexer.getVoltageAnalog());
        telemetry.addData("Actuator up?: ",actuator.isActivated());
        telemetry.update();
    }
}
