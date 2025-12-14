package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Indexer {

    public enum ArtifactColor { unknown, purple, green }

    public enum IndexerState {
        zero(0),
        one(1),
        two(2);

        public final int index;

        IndexerState(int index) {
            this.index = index;
        }

        public IndexerState next() {
            return values()[(this.index + 1) % 3];
        }
    }

    // config
    public static double offsetAngle = 17;
    public static double outtakeOffsetAngle = 5;
    public static double targetAngle = 0;

    // scan timing
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;

    // objects
    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl indexerServoControl;
    private final AnalogInput indexerAnalog;
    private final Actuator actuator;

    // internal state
    private IndexerState state = IndexerState.zero;
    private boolean intaking = true;

    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };

    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelay;

    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        indexerAnalog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        actuator = new Actuator(hardwareMap);
        indexerServoControl = new CRServoPositionControl(servo, indexerAnalog);
        colorSensor = new ColorSensorSystem(hardwareMap);
    }

    // getters
    public IndexerState getState() { return state; }
    public boolean getIntaking() { return intaking; }

    // Only use isBusy() when color sensing fully works
    public boolean isBusy() { return scanPending; }
    public double getActualMeasuredVoltage() { return indexerAnalog.getVoltage(); }

    public String getIntakingOrOuttaking() {
        return intaking ? "Intaking" : "Outtaking";
    }

    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state);
        }
    }
    // artifact color helpers
    public ArtifactColor stateToColor(IndexerState s) {
        return artifacts[s.index];
    }

    public void scanArtifact() {
        artifacts[state.index] = colorSensor.getColor();
    }

    // movement
    public void moveToColor(ArtifactColor color) {
        if (artifacts[0] == color) moveTo(IndexerState.zero);
        else if (artifacts[1] == color) moveTo(IndexerState.one);
        else if (artifacts[2] == color) moveTo(IndexerState.two);
    }

    public void moveTo(IndexerState newState) {

        if (newState == state && !scanPending) {
            return;
        }

        // Slot = 0, 1, 2
        //120 per position
        double wrappedTargetAngle = newState.index * 120.0;

        if (!intaking) {
            wrappedTargetAngle += outtakeOffsetAngle;
        }

        wrappedTargetAngle += offsetAngle;

        // Normalize to 0, 360
        wrappedTargetAngle %= 360.0;
        if (wrappedTargetAngle < 0) wrappedTargetAngle += 360.0;

        //they see me rolling
        double currentWrapped =
                indexerServoControl.getCurrentContinuousAngle() % 360.0;
        if (currentWrapped < 0) currentWrapped += 360.0;

        double delta = wrappedTargetAngle - currentWrapped;
        if (delta < 0) delta += 360.0; // CW distance

        double wait = Math.min(
                maxWait,
                Math.max(minWait, delta * msPerDegree)
        );

        scanTimer.reset();
        scanDelay = wait;
        scanPending = true;

        // ---------------- Command motion ----------------
        indexerServoControl.moveToAngle(wrappedTargetAngle);

        state = newState;
    }





    public void reset(Telemetry telem)
    {
        indexerServoControl.reset(telem);
    }

    public void update() {
        if (scanPending && scanTimer.milliseconds() >= scanDelay) {
            scanArtifact();
            scanPending = false;
        }
        indexerServoControl.update();

    }

    public IndexerState nextState() {
        return state.next();
    }
}
