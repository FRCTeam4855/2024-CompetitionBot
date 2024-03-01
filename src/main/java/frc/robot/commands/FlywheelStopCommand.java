package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStopCommand extends Command {

    private final FlywheelSubsystem Launcher;

    public FlywheelStopCommand(FlywheelSubsystem thisLauncherFlywheel) {
        Launcher = thisLauncherFlywheel;
    }

    public void initialize() {

    }

    public void execute() {
        Launcher.FlywheelStop();
    }

    public boolean isFinished() {
        return true;

    }
}
