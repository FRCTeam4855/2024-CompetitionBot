package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelLaunchCommand {
    
    private final FlywheelSubsystem Launcher;

    public FlywheelLaunchCommand(FlywheelSubsystem thisLauncherFlywheel) {
        Launcher = thisLauncherFlywheel;
    }

    public void initialize(){
        
    }

    public void execute(){
        Launcher.FlywheelLaunch();
    }

    public boolean isFinished(){
        return true;
        
    }
}
