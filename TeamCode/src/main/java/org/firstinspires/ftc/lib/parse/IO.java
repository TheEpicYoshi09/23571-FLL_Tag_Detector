package org.firstinspires.ftc.lib.parse;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import java.util.Collection;

public interface IO {
    @SuppressLint("DefaultLocale")
    default void putTelemetry(String dir, Collection<Object> data) {
        for (int i = 0; i < data.size(); i++) {
            telemetry.addData(String.format("%s.%d", dir, i), data.iterator().next());
        }
    }
}
