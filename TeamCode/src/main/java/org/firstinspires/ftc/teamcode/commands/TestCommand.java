package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class TestCommand implements Command {
    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
