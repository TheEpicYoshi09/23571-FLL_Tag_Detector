
package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public class LimeLight {
    public static final LimeLight INSTANCE = new LimeLight();
    private LimeLight() { }

    private Limelight3A limelight;

    private boolean I_AM_BLUE;
    private int BLUE_TAG_ID =  20;
    private int GPP_TAG_ID =  21;
    private int PGP_TAG_ID =  22;
    private int PPG_TAG_ID =  23;
    private int RED_TAG_ID =  24;

    private final static boolean LEDS = False;

    public void init(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // initialize ob_arr
        ob_arr.add(GPP_TAG_ID);
        ob_arr.add(PGP_TAG_ID);
        ob_arr.add(PPG_TAG_ID);
    }

    public void setAlliance(boolean iAmBlue) {
        I_AM_BLUE = iAmBlue;
        if (LEDS) {
            ledOn();
        }
    }

    public void ledOff() {
        if (LEDS) {
            ledBlue.setPower(0.);
            ledRed.setPower(0.);
        }
    }

    public void ledOn() {
        if (LEDS) {
            if (I_AM_BLUE) {
                ledBlue.setPower(1.);
            }
            else {
                ledRed.setPower(1.);
            }
        }
    }

    public int getObeliskTag() {
        int yaw_sign = I_AM_BLUE ? -1 : 1;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                Pose3D t_pose = fr.getTargetPoseCameraSpace()
                if (t_pose.rx * yaw_sign > 0) {
                    return fr.getFiducialId();
                }
            }
        }
        return -1;
    }

    public void stop() {
        limelight.stop();
    }

    public int getGreenSpot() {
       int i = getObeliskTag();
       if (i == GPP_TAG_ID) {return 0;}
       if (i == PGP_TAG_ID) {return 1;}
       if (i == PPG_TAG_ID) {return 2;}
       return -1;
    }

    // ftclib Pose2d with heading in radians
    public Pose2d getPose2d() {
        Pose2D pose2D = getPose2D();
        return new Pose2d(pose2D.getX(DistanceUnit.INCH),
                          pose2D.getY(DistanceUnit.INCH),
                          new Rotation2d(pose2D.getHeading(AngleUnit.RADIANS)));
    }

    // FTC Pose2D
    public Pose2D getPose2D() {
        Pose2D pose2D = new Pose2D();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            //Pose3D botpose = result.getBotpose();
            //Pose3D botposeMt2 = result.getBotpose_MT2();
            Pose3D pose3D = result.getBotpose_MT2();
            pose2D = new Pose2D(DistanceUnit.INCH,
                                pose3D.getX(DistanceUnit.INCH),
                                pose3D.getY(DistanceUnit.INCH),
                                AngleUnit.DEGREES,
				pose3D.getHeading(AngleUnit.DEGREES)));
        }
        return pose2D;
    }
}
