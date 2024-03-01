package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStartCommand extends Command {

    private final FlywheelSubsystem Launcher;

    public FlywheelStartCommand(FlywheelSubsystem thisLauncherFlywheel) {
        Launcher = thisLauncherFlywheel;
    }

    public void initialize() {

    }

    public void execute() {
        Launcher.FlywheelStart();
    }

    public boolean isFinished() {
        return true;

    }
}
