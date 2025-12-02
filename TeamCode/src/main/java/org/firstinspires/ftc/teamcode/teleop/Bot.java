package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
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

    public enum FSM {
        Intake,
        QuickOuttake,
        SortOuttake,
        Endgame
    }

    public FSM state;

    private static final double TRIGGER_DEADZONE = 0.05;
    private boolean quickSpinRequested = false;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        movement = new Movement(hardwareMap);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        telemetry = tele;
        state = FSM.Intake;
    }

    private void enterState(FSM newState) {
        if (state != newState) {
            state = newState;
            quickSpinRequested = false;
        }
    }

    public void teleopInit() {
        indexer.startIntake();
        enterState(FSM.Intake);
    }

    public void teleopTick() {
        g1.readButtons();
        g2.readButtons();

        handleMovement();

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentric = !fieldCentric;
        }

        switch (state) {
            case Intake:
                handleIntakeState();
                break;
            case QuickOuttake:
                handleQuickOuttakeState();
                break;
            case SortOuttake:
                handleSortOuttakeState();
                break;
            case Endgame:
                handleEndgameState();
                break;
        }

        // Telem (most sigma ever)
        telemetry.addData("Field Centric:", fieldCentric);
        telemetry.addData("Indexer States:", "Current State: %s , Next State: %s",indexer.getState(), indexer.nextState());
        telemetry.addData("Indexer Voltages:", "Target: %.3f , Actual: %.3f" , indexer.getTargetVoltage(), indexer.getVoltageAnalog());
        telemetry.addData("Actuator up?: ", actuator.isActivated());
        telemetry.update();
    }

    private void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();

        if (fieldCentric) movement.teleopTickFieldCentric(lx, ly, rx, 0, true);
        else movement.teleopTick(lx, ly, rx, 0);
    }

    private void handleIntakeState() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (leftTrigger > TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.A)) enterState(FSM.QuickOuttake);
        if (g2.wasJustPressed(GamepadKeys.Button.B)) enterState(FSM.SortOuttake);
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) enterState(FSM.Endgame);
    }

    private void handleQuickOuttakeState() {
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            quickSpinRequested = !quickSpinRequested;
            if (quickSpinRequested) {
                // TODO: start shooter quickly (e.g. outtake.startHighSpeed())
            } else {
                // TODO: outtake.stop()
            }
        }

        if (quickSpinRequested) {
            // TODO: do ts ig
            // Example: call indexer.advanceOne() with your own timing logic
        }

        if (g2.wasJustPressed(GamepadKeys.Button.A)) enterState(FSM.Intake);
    }

    private void handleSortOuttakeState() {
        // TODO: implement sorted shoot sequence:
        // 1) spin shooter to RPM
        // 2) wait for stable RPM (ideally ts is in a roadronere action)
        // 3) feed one ball at a time with small delays
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            // cancel sort and return to intake
            enterState(FSM.Intake);
        }
    }

    private void handleEndgameState() {
        // TODO: activate actuator to open slides or perform endgame actions
        // Example: actuator.extendSlides();
        if (g2.wasJustPressed(GamepadKeys.Button.A)) enterState(FSM.Intake);
    }
}
