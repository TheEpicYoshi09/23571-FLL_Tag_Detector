package org.firstinspires.ftc.teamcode.subsystems.intake.io;

import org.firstinspires.ftc.lib.parse.IO;

public interface IntakeIO {
    class IntakeIOInputs implements IO {
        public double intakeAngleDegrees = 0.0;
        public double intakeVelocityMetersPerSecond = 0.0;
        public double intakeAcceleration = 0.0;
    }

    void updateInputs(IntakeIOInputs inputs);

    void setAngleSetpoint(double degrees);

    void rotateFlywheels(double metersPerSecond);
}
