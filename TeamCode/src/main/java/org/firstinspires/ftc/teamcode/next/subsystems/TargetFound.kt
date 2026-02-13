package org.firstinspires.ftc.teamcode.next.subsystems
import com.qualcomm.ftccommon.SoundPlayer
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.hardwareMap


object TargetFound : Subsystem {
    val soundID = hardwareMap.appContext.resources.getIdentifier(
        "Relocate April Tag",
        "raw",
        hardwareMap.appContext.packageName
    )

}