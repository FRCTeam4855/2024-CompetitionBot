package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRetractCommand extends Command {
    private final ClimberSubsystem Climber;

    public ClimberRetractCommand(ClimberSubsystem thisClimber) {
        Climber = thisClimber;
    }

    public void initialize() {
    }

    public void execute() {
        Climber.ClimberRetract();
    }

    public boolean isFinished() {
        return true;
    }
}
