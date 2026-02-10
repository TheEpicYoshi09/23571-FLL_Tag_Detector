package org.firstinspires.ftc.team28420.types;

public class PolarVector {

    private final static double TWO_PI = 2 * Math.PI;
    private double theta, abs;

    public PolarVector(double theta, double abs) {
        this.theta = theta;
        this.abs = abs;
    }

    public PolarVector(Position pos) {
        this.theta = Math.atan2(pos.y, pos.x);
        this.abs = Math.hypot(pos.x, pos.y);
    }

    public static PolarVector fromPos(Position pos) {
        return new PolarVector(Math.atan2(pos.y, pos.x), Math.hypot(pos.x, pos.y));
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public double getAbs() {
        return abs;
    }

    public void setAbs(double abs) {
        this.abs = abs;
    }

    public PolarVector rotate(double angle) {
        theta += angle + TWO_PI;
        theta %= TWO_PI;
        return this;
    }
}
