package org.firstinspires.ftc.lib.hardware;

import lombok.Getter;

public class ActuatorConfig {

    public enum Mode {
        FLYWHEEL,
        SERVO
    }

    public enum StopMode {
        BRAKE,
        FLOAT
    }

    @Getter private StopMode stopMode = StopMode.FLOAT;
    @Getter private boolean inverted = false;
    @Getter private Mode mode = Mode.FLYWHEEL;
    @Getter private double kP = 0.0;
    @Getter private double kI = 0.0;
    @Getter private double kD = 0.0;
    @Getter private double kF = 0.0;
    @Getter private double radius = 0.0;

    public ActuatorConfig() {}

    public ActuatorConfig setStopMode(StopMode stopmode) {
        this.stopMode = stopmode;
        return this;
    }

    public ActuatorConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public ActuatorConfig setMode(Mode mode) {
        this.mode = mode;
        return this;
    }

    public ActuatorConfig setKP(double kP) {
        this.kP = kP;
        return this;
    }

    public ActuatorConfig setKI(double kI) {
        this.kI = kI;
        return this;
    }

    public ActuatorConfig setKD(double kD) {
        this.kD = kD;
        return this;
    }

    public ActuatorConfig setKF(double kF) {
        this.kF = kF;
        return this;
    }

    public ActuatorConfig setRadius(double radius) {
        this.radius = radius;
        return this;
    }

    public ActuatorConfig configure(
            boolean inverted,
            Mode mode,
            double kP,
            double kI,
            double kD,
            double kF,
            double radius
    ) {
        return this
                .setInverted(inverted)
                .setMode(mode)
                .setKP(kP)
                .setKI(kI)
                .setKD(kD)
                .setKF(kF)
                .setRadius(radius);
    }
}
