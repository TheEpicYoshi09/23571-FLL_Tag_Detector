package org.firstinspires.ftc.team28420.types;

public class MovementParams {
    private final PolarVector moveVector;
    private final double turnAbs;

    public MovementParams(PolarVector moveVector, double turnAbs) {
        this.moveVector = moveVector;
        this.turnAbs = turnAbs;
    }

    public PolarVector getMoveVector() {
        return moveVector;
    }

    public double getTurnAbs() {
        return turnAbs;
    }
}
