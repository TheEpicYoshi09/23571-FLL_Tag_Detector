package org.firstinspires.ftc.teamcode.subsystems.intake.io;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.lib.hardware.CyberMap;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;

public class IntakeIOMotor implements IntakeIO {
    private MotorEx pivotMaster;
    private MotorEx pivotSlave;
    private MotorEx flywheels;

    public IntakeIOMotor() {
        pivotMaster = new MotorEx(CyberMap.hardwareMap, IntakeConstants.kPivotMasterId);
        pivotMaster.setRunMode(Motor.RunMode.PositionControl);
        pivotMaster.setInverted(IntakeConstants.kPivotInverted);
        pivotMaster.setZeroPowerBehavior(IntakeConstants.kPivotIdleMode);
        pivotMaster.setPositionCoefficient(IntakeConstants.kPivotP);
        pivotMaster.setPositionTolerance(IntakeConstants.kPivotTolerance);

        pivotSlave = new MotorEx(CyberMap.hardwareMap, IntakeConstants.kPivotSlaveId);
        pivotSlave.setRunMode(Motor.RunMode.PositionControl);
        pivotSlave.setInverted(IntakeConstants.kPivotInverted);
        pivotSlave.setZeroPowerBehavior(IntakeConstants.kPivotIdleMode);
        pivotSlave.setPositionCoefficient(IntakeConstants.kPivotP);
        pivotSlave.setPositionTolerance(IntakeConstants.kPivotTolerance);

        flywheels = new MotorEx(CyberMap.hardwareMap, IntakeConstants.kFlywheelId);
        flywheels.setRunMode(Motor.RunMode.VelocityControl);
        flywheels.setInverted(IntakeConstants.kFlywheelsInverted);
        flywheels.setZeroPowerBehavior(IntakeConstants.kFlywheelIdleMode);
        flywheels.setVeloCoefficients(
                IntakeConstants.kFlywheelP,
                IntakeConstants.kFlywheelI,
                IntakeConstants.kFlywheelD
        );
        flywheels.setFeedforwardCoefficients(
                IntakeConstants.kFlywheelS,
                IntakeConstants.kFlywheelV
        );
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.intakeAngleDegrees = pivotMaster.getDistance();
        inputs.intakeVelocityMetersPerSecond = flywheels.getCorrectedVelocity();
        inputs.intakeAcceleration = flywheels.getAcceleration();
    }

    @Override
    public void setAngleSetpoint(double degrees) {
        pivotMaster.setTargetDistance(degrees);
        pivotSlave.setTargetDistance(degrees);
    }

    @Override
    public void rotateFlywheels(double metersPerSecond) {
        flywheels.setVelocity(metersPerSecond);
    }
}
