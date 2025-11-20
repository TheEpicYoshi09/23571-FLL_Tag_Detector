package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.Catapult;

public class Shooting {

    public void ShootCombo() {
        Vision.INSTANCE.myWaitForTarget();
        AutoDrive.INSTANCE.runSquareToTarget();
        Catapult.INSTANCE.launch();
    }

    public void Shoot(Catapult catapult) {
        catapult.launch();
    }

}
