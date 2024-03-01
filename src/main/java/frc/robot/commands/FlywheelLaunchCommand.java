package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelLaunchCommand extends Command{

    private final FlywheelSubsystem Launcher;

    public FlywheelLaunchCommand(FlywheelSubsystem thisLauncherFlywheel) {
        Launcher = thisLauncherFlywheel;
    }

    public void initialize() {

    }

    public void execute() {
        Launcher.FlywheelLaunch();
    }

    public boolean isFinished() {
        if (Launcher.FlywheelCheck() == true) {
            return true;
        } else {
            return false;
        }
    }
}