package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDropCommand extends Command {
    private final IntakeSubsystem Intake;

    public IntakeDropCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
        
    }

    public void execute() {
        Intake.IntakeOutput();
    }

    public boolean isFinished() {
        return true;
    }
}