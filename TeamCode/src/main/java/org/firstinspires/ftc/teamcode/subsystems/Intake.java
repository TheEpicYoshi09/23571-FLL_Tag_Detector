package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem  {
    private Intake() { }
    public static final Intake INSTANCE = new Intake();

    // USER CODE
    public String name = "intake_motor";
    private final double INTAKE_POWER_IN = 1.;
    private final double INTAKE_POWER_OUT = -.9;
    private final double INTAKE_POWER_OFF = 0.;

    public MotorEx motor;

    public Command resetZero() {
        return new InstantCommand(() -> { motor.setCurrentPosition(0.); });
    }
    public Command intake_Off() { return new InstantCommand(() -> {motor.setPower(INTAKE_POWER_OFF);});
    }
    public Command intake_on() {return new InstantCommand(() -> {motor.setPower(INTAKE_POWER_IN);});
    }
    public Command intake_out() {return new InstantCommand(() -> {motor.setPower(INTAKE_POWER_OUT);});
    }


    @Override
    public void initialize() {
        motor = new MotorEx(name);
    }

}
