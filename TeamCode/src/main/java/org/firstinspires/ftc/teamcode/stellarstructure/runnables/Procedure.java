package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Procedure extends Runnable {
    Directive[] directives;
    private int currentDirectiveIndex = 0;

    public Procedure(@NonNull Directive... directives) {
        if (directives.length == 0) {
            throw new IllegalArgumentException("No directives provided");
        }

        this.directives = directives;

        List<Subsystem> requiredSubsystems = new ArrayList<>();
        for (Directive directive : directives) {
			requiredSubsystems.addAll(Arrays.asList(directive.getRequiredSubsystems()));
        }
        setRequires(requiredSubsystems.toArray(new Subsystem[0]));
        setInterruptible(false);
    }

    @Override
    public void start(boolean hadToInterruptToStart) {
        directives[0].start(false);
    }

    @Override
    public void update() {
        // if the current directive is invalid or the whole procedure is done, do nothing
        if (currentDirectiveIndex < 0 || currentDirectiveIndex >= directives.length) {
            return;
        }

        Directive currentDirective = directives[currentDirectiveIndex];

        if (currentDirective.isFinished()) {
            currentDirective.stop(false);
            currentDirectiveIndex++;

            if (currentDirectiveIndex >= directives.length) {
                return;
            }

            currentDirective = directives[currentDirectiveIndex];
            currentDirective.start(false);

            return;
        }

        currentDirective.update();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted && currentDirectiveIndex >= 0 && currentDirectiveIndex < directives.length) {
            directives[currentDirectiveIndex].stop(true);
        }
    }

    @Override
    public boolean isFinished() {
        return currentDirectiveIndex >= directives.length;
    }
}
