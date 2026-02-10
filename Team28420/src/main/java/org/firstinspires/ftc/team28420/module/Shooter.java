package org.firstinspires.ftc.team28420.module;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team28420.processors.BallDetection;
import org.firstinspires.ftc.team28420.util.Config;
import org.opencv.core.Scalar;

import java.util.HashMap;

public class Shooter {
    public final DcMotorEx left, right, revolver;
    public final Servo pusher;
    public final ColorSensor cs;
    public final BallDetection.BallColor color = null;
    public final ElapsedTime shooterTime = new ElapsedTime();
    public DcMotorEx dribbler;
    public HashMap<String, Integer> sortSeqMap = null;
    public int globalTarget = 0;
    public ShooterState state = ShooterState.IDLE;
    public boolean correctMotif = false;
    public boolean manualControl = false;
    public boolean ballPresent = false;
    public boolean manualRotationActive = false;
    // TODO: DELETE "PG" LATER
    public String curMotif = "PG";

    public Shooter(HardwareMap hMap) {
        sortSeqMap = new HashMap<String, Integer>();
        sortSeqMap.put("PPG", 0);
        sortSeqMap.put("GPP", 1);
        sortSeqMap.put("PGP", 2);

        this.left = hMap.get(DcMotorEx.class, "shLeft");
        this.right = hMap.get(DcMotorEx.class, "shRight");
        this.revolver = hMap.get(DcMotorEx.class, "sort");
        this.cs = hMap.get(ColorSensor.class, "colorSensor");
        this.pusher = hMap.get(Servo.class, "pusher");
        dribbler = hMap.get(DcMotorEx.class, "dribbler");
    }

    public void setup() {
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        revolver.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
        revolver.setMode(mode);
    }

    public void setMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left.setZeroPowerBehavior(behavior);
        right.setZeroPowerBehavior(behavior);
    }

    public void syncTicks() {
        globalTarget = revolver.getCurrentPosition();
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("ANGLE", currentAngle());
        telemetry.addData("MOTIF", curMotif);
        telemetry.addData("CORRECT MOTIF", correctMotif);
        telemetry.addData("SHOOTING ALLOWED", isShootable());
    }

    public void snapToNearestSlot() {
        double ticksPerTurn = Config.ShooterConf.SORT_MOTOR_TICKS_PER_TURN;
        double ticksPerSlot = ticksPerTurn / 3.0;
        double offsetTicks = (60.0 * ticksPerTurn) / 360.0;

        globalTarget = (int) (Math.round((globalTarget - offsetTicks) / ticksPerSlot) * ticksPerSlot + offsetTicks);

        revolver.setTargetPosition(globalTarget);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setPower(Config.ShooterConf.SORT_MOTOR_POWER);
    }

    // 360 = ticks_turn
    // 360 / 120 = ticks_turn / x
    // x = target_deg * ticks_turn / 360
    public void rotateRevolver(double deg) {
        globalTarget += (int) (deg * Config.ShooterConf.SORT_MOTOR_TICKS_PER_TURN / 360.0);
        revolver.setTargetPosition(globalTarget);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setPower(Config.ShooterConf.SORT_MOTOR_POWER);
    }

    public void scanBall() {
        if (curMotif.length() == 3) return;

        DistanceSensor sensorDistance = (DistanceSensor) cs;
        double distanceInCm = sensorDistance.getDistance(DistanceUnit.CM);
        Config.Etc.telemetry.addData("distance", distanceInCm);

        NormalizedColorSensor normalizedSensor = (NormalizedColorSensor) cs;
        NormalizedRGBA colors = normalizedSensor.getNormalizedColors();

        // вяртае ад 0 да 1
        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        float[] hsv = {0F, 0F, 0F};

        int scale = 255;
        Color.RGBToHSV((int) (red * scale), (int) (green * scale), (int) (blue * scale), hsv);
        Config.Etc.telemetry.addData("hue", hsv[0]);
        Config.Etc.telemetry.addData("saturation", hsv[1]);
        Config.Etc.telemetry.addData("value", hsv[2]);

        BallDetection.BallColor color = null;
        if (checkColors(hsv, Config.BallDetectionConf.cslowPurple, Config.BallDetectionConf.cshighPurple)) {
            color = BallDetection.BallColor.PURPLE;
        } else if (checkColors(hsv, Config.BallDetectionConf.cslowGreen, Config.BallDetectionConf.cshighGreen)) {
            color = BallDetection.BallColor.GREEN;
        } else color = null;

        if (distanceInCm <= 4 && color != null) {
            if (!ballPresent) {
                if (curMotif.length() < 3) {
                    if (color == BallDetection.BallColor.PURPLE) {
                        curMotif += 'P';
                    }
                    if (color == BallDetection.BallColor.GREEN) {
                        curMotif += 'G';
                    }
                    ballPresent = true;

                    if (curMotif.length() == 3) {
                        int g = 0, p = 0;
                        for (int i = 0; i < 3; i++) {
                            if (curMotif.charAt(i) == 'G') g++;
                            if (curMotif.charAt(i) == 'P') p++;
                        }
                        if (g == 1 && p == 2) {
                            Config.Etc.telemetry.addData("g", g);
                            Config.Etc.telemetry.addData("p", p);
                            correctMotif = true;
                            int ourIndex = sortSeqMap.get(curMotif);
                            int targetIndex = sortSeqMap.get(Config.ShooterConf.TARGET_MOTIF);
                            curMotif = Config.ShooterConf.TARGET_MOTIF;
                            int move = (targetIndex - ourIndex + 3) % 3;
                            rotateRevolver(60);
                            // TODO: calibrate
                            if (move == 1) rotateRevolver(120);
                            if (move == 2) rotateRevolver(-120);
                        } else {
                            correctMotif = false;
                            Config.Etc.telemetry.addLine("Взят плохой мотив, сбрось мячи");
                        }
                    } else {
                        rotateRevolver(120);
                    }
                }
            }
        } else {
            ballPresent = false;
        }
    }

    public boolean checkColors(float[] hsv, Scalar low, Scalar high) {
        boolean hueMatch;

        if (low.val[0] > high.val[0]) {
            // This handles the Red wrap-around (e.g., low is 350, high is 10)
            hueMatch = (hsv[0] >= low.val[0] || hsv[0] <= high.val[0]);
        } else {
            // Standard range check
            hueMatch = (hsv[0] >= low.val[0] && hsv[0] <= high.val[0]);
        }

        return hueMatch && (hsv[1] >= low.val[1] && hsv[1] <= high.val[1]) && (hsv[2] >= low.val[2] && hsv[2] <= high.val[2]);
    }

    public boolean isShootable() {
        double currentAngle = currentAngle() % 360;
        if (currentAngle < 0) currentAngle += 360;

        boolean nearSlot1 = Math.abs(currentAngle - 60) < 5;
        boolean nearSlot2 = Math.abs(currentAngle - 180) < 5;
        boolean nearSlot3 = Math.abs(currentAngle - 300) < 5;

        return (nearSlot1 || nearSlot2 || nearSlot3) && !revolver.isBusy();
    }

    public void sortedNextBall() {
        if (!revolver.isBusy()) {
            if (curMotif.length() > 0) {
                curMotif = curMotif.substring(0, curMotif.length() - 1);
                if (curMotif.length() > 0) rotateRevolver(-120);
                else {
                    rotateRevolver(-60);
                    // калі матываў няма
                    correctMotif = false;
                }
            }
        }
    }

    public void pushBall(boolean push) {
        if (push) {
            pusher.setPosition(0);
        } else {
            pusher.setPosition(0.5);
        }
    }

    public double currentAngle() {
        return revolver.getCurrentPosition() / Config.ShooterConf.SORT_MOTOR_TICKS_PER_TURN * 360.0;
    }

    public void shooterRun(float coef) {
        left.setVelocity(Config.ShooterConf.VELOCITY * coef * coef);
        right.setVelocity(Config.ShooterConf.VELOCITY * coef * coef);
    }

    public void shooterStop() {
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public enum ShooterState {IDLE, SHOOTING, STOP_SHOOTING, REVOLVER_TURNING}
}
