package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStopCommand extends Command{
    private final DriveSubsystem Drive;

    public DriveStopCommand(DriveSubsystem thisDrive) {
        Drive = thisDrive;
    }

    public void initialize() {
        
    }

    public void execute() {
        Drive.setAutoStop();
    }

    public boolean isFinished() {
            return true;
        }
}
