package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStartCommand extends Command {

    private final FlywheelSubsystem Launcher;
    private int wheelSpeed;
    private static boolean waitForIt;

    public FlywheelStartCommand(FlywheelSubsystem thisLauncherFlywheel, int desiredWheelSpeed, boolean waitCheck) {
        Launcher = thisLauncherFlywheel;
        wheelSpeed = desiredWheelSpeed;
        waitForIt = waitCheck;
    }

    public void initialize() {
        Launcher.FlywheelStart(wheelSpeed);
    }

    public void execute() {
    }

    public boolean isFinished() {
        //return true;
         if(!waitForIt) {
                System.out.println("Did not request to wait for flywheel to start");
                return true;
         } else {
            System.out.println("Requested to wait for flywheel to start");
             return Launcher.FlywheelCheck(wheelSpeed);
         }
    }
}
