package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStartCommand extends Command {

    private final FlywheelSubsystem Launcher;

    public FlywheelStartCommand(FlywheelSubsystem thisLauncherFlywheel) {
        Launcher = thisLauncherFlywheel;
        //addRequirements(thisLauncherFlywheel);
    }

    public void initialize() {
        Launcher.FlywheelStart();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return Launcher.FlywheelCheck();
    }
}
