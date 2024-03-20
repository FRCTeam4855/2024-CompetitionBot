package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class LimelightStrafeCommand extends Command {
    Limelight limelight;
    DriveSubsystem drive;

    public LimelightStrafeCommand(DriveSubsystem thisDrive, Limelight thisLimelight) {
        drive = thisDrive;
        limelight = thisLimelight;
    }

    public void initialize() {

    }

    public void execute() {
        if (limelight.xvalue >= 4) {
            drive.strafeRight();
        } else if (limelight.xvalue <= 0) {
            drive.strafeLeft();
        }
    }

    public boolean isFinished() {
        if (limelight.xvalue < 3 && limelight.xvalue > 1) {
            drive.setStop();
            return true;
        } else {
            return false;
        }
    }
}
