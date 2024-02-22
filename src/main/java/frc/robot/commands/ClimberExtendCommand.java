package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends Command {
    private final ClimberSubsystem Climber;

    public ClimberExtendCommand(ClimberSubsystem thisClimber){
         Climber = thisClimber;
    }
    public void initialize() {}

    public void execute() {
        Climber.ClimberExtend();
    }

    public boolean isFinished() {
        return true;
      }
}