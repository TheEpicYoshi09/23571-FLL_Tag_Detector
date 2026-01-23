package org.firstinspires.ftc.lib.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public interface CyberActuator {
    void apply(ActuatorConfig config);

    void setSetpoint(Velocity velocity);

    void setSetpoint(AngularVelocity velocity);

    void setSetpoint(int encoderPosition);

    void setSetpoint(double proportion);

    int getPosition();
}
