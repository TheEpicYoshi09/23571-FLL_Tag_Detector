package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

public class ShootingController {

    public enum ShootState {
        IDLE,
        WAIT_FOR_SPINUP,
        NEXT_ARTIFACT,
        FIRE,
        RETRACT,
        FINISH,
    }

    private final RobotHardware robot;
    private final FlywheelController flywheelController;
    private final SpindexerController spindexerController;
    private final Telemetry telemetry;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private ShootState shootState = ShootState.IDLE;

    public ShootingController(RobotHardware robot, FlywheelController flywheelController, SpindexerController spindexerController, Telemetry telemetry) {
        this.robot = robot;
        this.flywheelController = flywheelController;
        this.spindexerController = spindexerController;
        this.telemetry = telemetry;
    }

    public void startShootSequence() {
        shootTimer.reset();
        shootState = ShootState.WAIT_FOR_SPINUP;
        robot.kicker.setPosition(Constants.KICKER_DOWN);
    }

    public void update() {
        if (shootState == ShootState.IDLE || spindexerController.isSpindexerEmpty()) {
            return;
        }

        if (!flywheelController.isEnabled() || flywheelController.getTargetRpm() <= 0) {
            robot.kicker.setPosition(Constants.KICKER_DOWN);
            shootState = ShootState.IDLE;
            return;
        }

        switch (shootState) {
            case WAIT_FOR_SPINUP:
                if (flywheelController.isAtSpeed() && isAimedAtTarget() && shootTimer.milliseconds() >= 200) {
                    robot.kicker.setPosition(Constants.KICKER_UP);
                    shootTimer.reset();
                    shootState = ShootState.FIRE;
                }
                break;
            case NEXT_ARTIFACT:
                if (shootTimer.milliseconds() >= Constants.SHOOT_RETRACT_DURATION_MS) {
                    spindexerController.advanceSpindexer();
                    if (spindexerController.getCurrentSlotState() != ArtifactTracker.SlotStatus.VACANT) {
                        shootState = ShootState.FIRE;
                    }
                }
                break;
            case FIRE:
                if (shootTimer.milliseconds() >= Constants.SHOOT_FIRE_DURATION_MS) {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                    shootTimer.reset();
                    shootState = ShootState.RETRACT;
                }
                break;
            case RETRACT:
                if (shootTimer.milliseconds() >= Constants.SHOOT_RETRACT_DURATION_MS) {
                    if (!spindexerController.isSpindexerEmpty()) {
                        shootTimer.reset();
                        shootState = ShootState.NEXT_ARTIFACT;
                    } else {
                        shootTimer.reset();
                        shootState = ShootState.FINISH;
                    }
                }
                break;
            case FINISH:
                if (shootTimer.milliseconds() >= 300) {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                    shootState = ShootState.IDLE;
                }
                break;
            default:
                shootState = ShootState.IDLE;
                spindexerController.setPosition(0);
                break;
        }

        telemetry.addData("Shoot State", shootState);
    }

    /**
     * Update the shooting sequence and report when the full firing cycle has finished.
     *
     * @return true once the controller has completed the queued shots and returned to IDLE
     */
    public boolean updateAndIsComplete() {
        update();
        return shootState == ShootState.IDLE && spindexerController.isSpindexerEmpty();
    }

    public boolean isIdle() {
        return shootState == ShootState.IDLE;
    }

    public ShootState getShootState() {
        return shootState;
    }

    private boolean isAimedAtTarget() {
        LLResult result = robot.getLatestLimelightResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        double txPercent = result.getTx();
        if (!Double.isNaN(txPercent)) {
            telemetry.addData("*** TX PERCENT", Math.abs(txPercent));
            return Math.abs(txPercent) <= 0.5; // USE TO BE <=, AND 0.075
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return false;
        }

        double txDegrees = fiducials.get(0).getTargetXDegrees();
        telemetry.addData("*** TX DEGREES", txDegrees);
        return Math.abs(txDegrees) <= 10.0;
    }
}
