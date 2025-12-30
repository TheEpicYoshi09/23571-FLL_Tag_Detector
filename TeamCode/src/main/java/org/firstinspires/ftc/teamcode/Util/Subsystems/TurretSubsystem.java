package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.OpModes.NextFTCTeleop;
import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;

    MotorEx launcher = new MotorEx(UniConstants.LAUNCHER_STRING).floatMode();

    int targetVelocity = 0;
    public static ControlSystem launcherControl;
    public static double p = .00022, i = 0, d = 0;

    boolean usingTurret = false;
    MotorEx turret;
    public static double turretTargetAngle = 0;
    public static double turretCurrentPos = 0;
    private PDFLController turretControl;
    public static double pTurret = 0, dTurret = 0, lTurret = 0, fTurret = 0;


    public TurretSubsystem(){}



    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());


        if(usingTurret){
            turret = new MotorEx(UniConstants.TURRET_STRING).floatMode().zeroed();
            turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
        }

    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        launcherControl = ControlSystem.builder()
                .velPid(p, i, d)
                .build();
        launcherControl.setLastMeasurement(new KineticState(0, launcher.getVelocity() * 2.1));
        launcher.setPower(launcherControl.calculate(new KineticState(0, targetVelocity)));

        if(usingTurret) {
            turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
            turretCurrentPos = turret.getCurrentPosition();
            turretControl.update(turretCurrentPos);
            turret.setPower(turretControl.runPDFL(2));
        }
    }



    public Command commandRunToVelocity(double velocity){
        return new RunToPosition(launcherControl, velocity);
    }


    public  double getTargetVelocity(double distanceToGoalInMeters) {
        //https://www.desmos.com/calculator/yw7iis7m3w
        //https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
        return Math.sqrt(
                ((9.81) * (Math.pow(distanceToGoalInMeters, 2)))
                        /
                        (Math.pow(2 * (Math.cos(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))), 2) * ((distanceToGoalInMeters * Math.tan(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))) - UniConstants.HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS))
        );
    }

    public void setTargetVelocity(int velo){
        targetVelocity = velo;
    }

    //Uses degrees
    public double angleToTicks(double angle){
        return angle * UniConstants.TURRET_TICKS_PER_DEGREE;
    }

    //Uses degrees
    public double ticksToAngle(double ticks){
        return (ticks / UniConstants.TURRET_TICKS_PER_DEGREE) % 360;
    }

    public double getCurrentVelocity(){
        return Math.abs(launcher.getVelocity() * 2.1);
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addData("Target Velocity ", targetVelocity);
                telemetry.addData("Current Velocity ", getCurrentVelocity());
                if(usingTurret) {
                    telemetry.addLine();
                    telemetry.addData("Turret Position Deg ", ticksToAngle(turretCurrentPos));
                    telemetry.addData("Turret Target Deg ", turretTargetAngle);
                }

                telemetry.addLine("END OF OUTTAKE LOG");
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }


}
