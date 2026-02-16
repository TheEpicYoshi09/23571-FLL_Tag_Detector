package org.firstinspires.ftc.teamcode.next.subsystems
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.hardware.DcMotor
import com.sun.tools.doclint.Entity
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool
import org.firstinspires.ftc.robotcore.internal.android.SoundPoolIntf
import java.lang.Math.pow
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sin
import kotlin.math.sqrt

@TeleOp
class TeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent( TargetFound),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    var targetAcquired = false
    var Dist1 = 0.0
    var Dist2 = 0.0
    var Theta = 0.0
    var Distance = 0.0
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private var aprilTag: AprilTagProcessor? = null

    /**
     * The variable to store our instance of the vision portal.
     */
    private var visionPortal: VisionPortal? = null

    override fun runOpMode() {
        initAprilTag()

        aS.initialize(SoundPlayer.getInstance())

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Touch START to start OpMode")
        telemetry.update()
        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag()

                if(targetAcquired == false){
                    aS.play("Relocate April tag")
                    sleep(1000)
                }
                // Push telemetry to the Driver Station.
                telemetry.update()
                // Share the CPU.
            }
        }

        aS.close()

        // Save more CPU resources when camera is no longer needed.
        visionPortal!!.close()
    } // end method runOpMode()
    private fun initAprilTag() {
        // Create the AprilTag processor.

        aprilTag =
            AprilTagProcessor.Builder() // The following default settings are available to un-comment and edit as needed.

                .build()

        // Create the vision portal by using a builder.
        val builder = VisionPortal.Builder()

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "The Eye"))
        // Set and enable the processor.
        builder.addProcessor(aprilTag)

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build()

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal!!.resumeStreaming()
    } // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    val aS by lazy { AndroidSoundPool() }

    private fun telemetryAprilTag() {
        val currentDetections: MutableList<AprilTagDetection> = aprilTag!!.getDetections()

        for (detection in currentDetections) {
            aS.play("AT located calculating new directions.mp3")
            //Dist1 = detection.ftcPose.y*sin(detection.ftcPose.yaw)
            //Dist2 = detection.ftcPose.y*cos(detection.ftcPose.yaw)
            //Theta = detection.ftcPose.yaw
            //val d = sqrt(pow(Dist1, 2.0) + pow(Dist2, 2.0))
            //val d2T = Dist2 * sin(Theta)  / Dist1
            //val targetAngle = 180.0 - d2T - Theta
           // telemetry.addData("Target angle", targetAngle)
            Distance = detection.ftcPose.y/12
            sleep(4000)
            aS.play("Move Forward.mp3")
            sleep(2000)

            val t: String = ( when(floor(Distance / 12).toInt()) {
                0 -> {"ignore"}
                1 -> {"1 steps.mp3"}
                2 -> {"2 steps.mp3"}
                3 -> {"3 steps.mp3"}
                4 -> {"4 steps.mp3"}
                5 -> {"5 steps.mp3"}
                6 -> {"6 steps.mp3"}
                7 -> {"7 steps.mp3"}
                8 -> {"8 steps.mp3"}
                9 -> {"9 steps.mp3"}
                else -> {"ignore"}
            }
                    )
            if(t != "ignore") {
                aS.play(t)
            }
        }
        aS.volume = 100.0f
        telemetry.addData(" AprilTags Detected", currentDetections.size)
        telemetry.addData(" = Distance To Target", Distance)
        //telemetry.addLine(String.format ("Opposite = %.1f", Dist1))
        //telemetry.addLine(String.format ("Ajacent = %.1f", Dist2))
        //telemetry.addLine(String.format ("Theta = %.2f", Theta))

        var count = 0
        while(count < 5) {
            if (currentDetections.isNotEmpty()) {
                count++
                sleep(5000)
                targetAcquired = true
            } else {
                targetAcquired = false
                break;
            }
        }

        // Add "key" information to telemetry
    } // end method telemetryAprilTag()

} // end class