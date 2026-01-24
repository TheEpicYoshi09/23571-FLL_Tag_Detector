package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.intake.io.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.io.IntakeIOMotor;

import java.util.ArrayList;

import lombok.Getter;
import lombok.Setter;

public class IntakeSubsystem implements Subsystem {
    private IntakeIO io;
    private IntakeIO.IntakeIOInputs inputs;

    @Getter @Setter
    private double flywheelsSetpoint = 0.0;
    @Getter @Setter
    private double pivotSetpoint = 0.0;

    private static IntakeSubsystem m_instance;

    public IntakeSubsystem getInstance() {
        if (m_instance == null) m_instance = new IntakeSubsystem();
        return m_instance;
    }

    private IntakeSubsystem() {
        io = new IntakeIOMotor();
        inputs = new IntakeIO.IntakeIOInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        ArrayList<Object> collection = new ArrayList<>();
        collection.add(inputs.intakeAngleDegrees);
        collection.add(inputs.intakeVelocityMetersPerSecond);
        collection.add(inputs.intakeAcceleration);
        inputs.putTelemetry("Intake", collection);

        io.rotateFlywheels(flywheelsSetpoint);
        io.setAngleSetpoint(pivotSetpoint);
    }
}
