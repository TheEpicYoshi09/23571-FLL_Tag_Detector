package org.firstinspires.ftc.teamcode.next.subsystems
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import com.qualcomm.ftccommon.SoundPlayer
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import kotlin.math.cos
import kotlin.math.sin

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

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Touch START to start OpMode")
        telemetry.update()
        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag()

                // Push telemetry to the Driver Station.
                telemetry.update()
                // Share the CPU.
                sleep(20)
            }
        }

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
    private fun telemetryAprilTag() {
        val currentDetections: MutableList<AprilTagDetection> = aprilTag!!.detections
        for (detection in currentDetections) {
            Dist1 = detection.ftcPose.y*sin(detection.ftcPose.yaw)
            Dist2 = detection.ftcPose.y*cos(detection.ftcPose.yaw)
        }
        if (currentDetections.isNotEmpty()) {
            targetAcquired = true
        } else {
            targetAcquired = false
        }
        telemetry.addData(" AprilTags Detected", currentDetections.size)

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
        telemetry.addLine("RBE = Range, Bearing & Elevation")
        telemetry.addLine(String.format ("Dist1 = %.2f", Dist1))
        telemetry.addLine(String.format ("Dist2 = %.2f", Dist2))
    } // end method telemetryAprilTag()
} // end class