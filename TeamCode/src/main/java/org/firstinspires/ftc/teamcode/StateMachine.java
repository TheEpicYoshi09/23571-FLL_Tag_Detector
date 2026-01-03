package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FindGoal;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.HashMap;
import java.util.Map;
import java.util.zip.DeflaterOutputStream;

public class StateMachine {
    public enum State {
        HOME,
        AUTO_HOME_NEAR,
        AUTO_NEAR,
        AUTO_HOME_FAR,
        AUTO_FAR,
        AUTO_LEAVE_NEAR,
        AUTO_LEAVE_FAR,
    }

    public enum AUTO_PATHS {
        NEAR_PATH_TO_SHOOT_AREA,
        NEAR_SHOOT_AREA_TO_SPIKE1,
        NEAR_GOTO_SHOOT_SPIKE1,
        NEAR_PICKUP_SPIKE1,
        FAR_START_TO_SHOOT,
        FAR_SHOOT_TO_SPIKE3_LINEUP,
        FAR_SPIKE3_PICKUP_PART1,
        FAR_SPIKE3_PICKUP_PART2,
        FAR_SPIKE3_TO_SHOOT,
        FAR_SHOOT_LEAVE,

        NEAR_SHOOT_TO_WALL,
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;
    private final ShootingController shootingController;
    private final FlywheelController flywheelController;
    private final TurretTracker turretTracker;
    private final FindGoal findGoal;

    private Timer pathTimer = new Timer();
    private Timer autoTimer = new Timer();
    private int autoNearSubStep = 0;
    private int autoFarSubStep = 0;
    private boolean shootStarted = false;

    private final double[] spindexerPositions = new double[]{org.firstinspires.ftc.teamcode.Constants.spindexer1, org.firstinspires.ftc.teamcode.Constants.spindexer2, org.firstinspires.ftc.teamcode.Constants.spindexer3};
    private int spindexerIndex = 0;

    private Map<AUTO_PATHS, PathChain> paths = new HashMap<>();

    public StateMachine(RobotHardware hardware, Follower follower, ShootingController shootingController,
                       FlywheelController flywheelController, TurretTracker turretTracker) {
        this.robot = hardware;
        this.follower = follower;
        this.shootingController = shootingController;
        this.flywheelController = flywheelController;
        this.turretTracker = turretTracker;
        this.findGoal = new FindGoal(hardware);
        setState(State.HOME, true);
    }
    public void setState(State state, boolean doUpdate) {
        currentState = state;

        // reset blah blah

        pathTimer.resetTimer();

        autoNearSubStep = 0;
        autoFarSubStep = 0;

        shootStarted = false;

        if (doUpdate) { update(); }
    }

    public void setState(State state) {
        setState(state, false);
    }

    public State getState() { return currentState; }

    private void buildPath(AUTO_PATHS name, Pose blueFrom, Pose blueTo, Pose redFrom, Pose redTo) {
        Pose from = robot.allianceColorBlue ? blueFrom : redFrom;
        Pose to = robot.allianceColorBlue ? blueTo : redTo;
        paths.put(name, Constants.buildPath(follower, from, to));
    }

    public void init() {
        /// NEAR
        buildPath(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_START, DecodePaths.BLUE_NEAR_SHOOT,
                DecodePaths.RED_NEAR_START, DecodePaths.RED_NEAR_SHOOT);

        buildPath(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_GOTO_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_PICKUP_SPIKE1,
                DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS, DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS,
                DecodePaths.RED_NEAR_GOTO_ARTIFACTS, DecodePaths.RED_NEAR_PICKUP_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1,
                DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS, DecodePaths.BLUE_NEAR_GO_INSIDE_ZONE,
                DecodePaths.RED_NEAR_PICKUP_ARTIFACTS, DecodePaths.RED_NEAR_GO_INSIDE_ZONE);

        /// FAR
        buildPath(AUTO_PATHS.FAR_START_TO_SHOOT,
                DecodePaths.BLUE_FAR_START, DecodePaths.BLUE_FAR_SHOOT,
                DecodePaths.RED_FAR_START, DecodePaths.RED_FAR_SHOOT);

        buildPath(AUTO_PATHS.FAR_SHOOT_TO_SPIKE3_LINEUP,
                DecodePaths.BLUE_FAR_SHOOT, DecodePaths.BLUE_FAR_SHOOT_TO_SPIKE3,
                DecodePaths.RED_FAR_SHOOT, DecodePaths.RED_FAR_SHOOT_TO_SPIKE3);

        buildPath(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART1,
                DecodePaths.BLUE_FAR_SHOOT_TO_SPIKE3, DecodePaths.BLUE_FAR_PICKUP_SPIKE3_PART1,
                DecodePaths.RED_FAR_SHOOT_TO_SPIKE3, DecodePaths.RED_FAR_PICKUP_SPIKE3_PART1);

        buildPath(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART2,
                DecodePaths.BLUE_FAR_PICKUP_SPIKE3_PART1, DecodePaths.BLUE_FAR_PICKUP_SPIKE3_PART2,
                DecodePaths.RED_FAR_PICKUP_SPIKE3_PART1, DecodePaths.RED_FAR_PICKUP_SPIKE3_PART2);

        buildPath(AUTO_PATHS.FAR_SPIKE3_TO_SHOOT,
                DecodePaths.BLUE_FAR_PICKUP_SPIKE3_PART2, DecodePaths.BLUE_FAR_SHOOT,
                DecodePaths.RED_FAR_PICKUP_SPIKE3_PART2, DecodePaths.RED_FAR_SHOOT);

        buildPath(AUTO_PATHS.FAR_SHOOT_LEAVE,
                DecodePaths.BLUE_FAR_SHOOT, DecodePaths.BLUE_FAR_LEAVE,
                DecodePaths.RED_FAR_SHOOT, DecodePaths.RED_FAR_LEAVE
        );

        /// NEAR LEAVE
        buildPath(AUTO_PATHS.NEAR_SHOOT_TO_WALL,
                DecodePaths.BLUE_NEAR_START, DecodePaths.BLUE_NEAR_FROM_SHOOT_TO_WALL,
                DecodePaths.RED_NEAR_START, DecodePaths.RED_NEAR_FROM_SHOOT_TO_WALL
        );;
    }

    private boolean shoot(PathChain breakPath) {
        if (breakPath != null) {
            breakOutAuto(breakPath);
        }

        if (shootingController == null) {
            return true;
        }

        if (turretTracker != null) {
            robot.refreshLimelightResult();
            turretTracker.update();
        }

        if (flywheelController != null) {
            flywheelController.update();
        }

        if (!shootStarted
                && flywheelController != null
                && flywheelController.isEnabled()
                && flywheelController.getTargetRpm() > 0) {
            shootingController.startShootSequence();
            shootStarted = true;
        }

        boolean finishedShooting = shootStarted && shootingController.updateAndIsComplete();
        if (finishedShooting) {
            shootStarted = false;
            return true;
        }
        return false;
    }

    private void runFlywheel() {
        if (flywheelController != null) {
            if (!flywheelController.isEnabled()) {
                flywheelController.toggle();
            }
            flywheelController.update();
        }
    }

    private void stopFlywheel() {
        if (flywheelController != null && flywheelController.isEnabled()) {
            flywheelController.toggle();
        }
    }

    private void setSpindexPosition(int position) {
        spindexerIndex = position;
        robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
        robot.spindexerPos = spindexerPositions[spindexerIndex];
    }

    private void breakOutAuto(PathChain breakPath) {
        if (autoTimer.getElapsedTimeSeconds() >= 29.1) {            setState(State.HOME, true);
            robot.runIntake(RobotHardware.IntakeDirection.STOP);
            stopFlywheel();
            follower.followPath(breakPath);
        }
    }

    public void update() {
        switch (currentState) {
            case HOME:
                setSpindexPosition(0);
                break;
            case AUTO_HOME_NEAR:
                Pose startPoseNear = robot.allianceColorBlue ? DecodePaths.BLUE_NEAR_START : DecodePaths.RED_NEAR_START;
                follower.setStartingPose(startPoseNear);
                follower.setPose(startPoseNear);
                break;
            case AUTO_HOME_FAR:
                Pose startPoseFar = robot.allianceColorBlue ? DecodePaths.BLUE_FAR_START : DecodePaths.RED_FAR_START;
                follower.setStartingPose(startPoseFar);
                follower.setPose(startPoseFar);
                break;
            case AUTO_NEAR:
                switch (autoNearSubStep) {
                    case 0:
                        autoTimer.resetTimer();
                        runFlywheel();
                        follower.followPath(paths.get(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA), true);
                        autoNearSubStep++;
                        break;
                    case 1:
                        boolean turretReady = findGoal.updateAndIsDone();
                        if (turretReady) {
                            autoNearSubStep++;
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            boolean shot = shoot(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1));
                            if (shot) {
                                autoNearSubStep++;
                            }
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            stopFlywheel();
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1), true);
                            setSpindexPosition(0);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                            robot.runIntake(RobotHardware.IntakeDirection.IN);
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_SPIKE1), true);
                            runFlywheel();
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 5:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1) {
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1), true);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 6:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                            robot.runIntake(RobotHardware.IntakeDirection.STOP);
                            autoNearSubStep++;
                        }
                        break;
                    case 7:
                        if (!follower.isBusy()) {
                            boolean shot = shoot(null);
                            if (shot) {
                                stopFlywheel();
                                autoNearSubStep++;
                            }
                        }
                        break;
                    default:
                        break;
                }
                break;
            case AUTO_FAR:
                switch (autoFarSubStep) {
                    case 0:
                        autoTimer.resetTimer();
                        follower.followPath(paths.get(AUTO_PATHS.FAR_START_TO_SHOOT), true);
                        runFlywheel();
                        autoFarSubStep++;
                        break;
                    case 1:
                        boolean turretReady = findGoal.updateAndIsDone();
                        if (turretReady) {
                            autoFarSubStep++;
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            boolean shot = shoot(paths.get(AUTO_PATHS.FAR_SHOOT_LEAVE));
                            if (shot) {
                                follower.followPath(paths.get(AUTO_PATHS.FAR_SHOOT_TO_SPIKE3_LINEUP), true);
                                setSpindexPosition(2);
                                robot.runIntake(RobotHardware.IntakeDirection.IN);
                                autoFarSubStep++;
                            }
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            follower.followPath(paths.get(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART1), true);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 4:
                        if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() >= 1) {
                            setSpindexPosition(0);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 5:
                        if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() >= 1) {
                            //setSpindexPosition(2);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 6:
                        if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() >= 1.375) {
                            follower.followPath(paths.get(AUTO_PATHS.FAR_SPIKE3_PICKUP_PART2), true);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 7:
                        if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() >= 1.45) {
                            robot.runIntake(RobotHardware.IntakeDirection.STOP);
                            follower.followPath(paths.get(AUTO_PATHS.FAR_SPIKE3_TO_SHOOT), true);
                            autoFarSubStep++;
                        }
                        break;
                    case 8:
                        if (!follower.isBusy()) {
                            boolean shot2 = shoot(paths.get(AUTO_PATHS.FAR_SHOOT_LEAVE));
                            if (shot2) {
                                follower.followPath(paths.get(AUTO_PATHS.FAR_SHOOT_LEAVE), true);
                                stopFlywheel();
                                autoFarSubStep++;
                            }
                        }
                        break;
                    default:
                        break;
                }
                break;
            case AUTO_LEAVE_NEAR:
                follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_TO_WALL), true);
                break;
            case AUTO_LEAVE_FAR:
                follower.followPath(paths.get(AUTO_PATHS.FAR_SHOOT_LEAVE), false);
                break;
            default:
                break;
        }
    }
}
