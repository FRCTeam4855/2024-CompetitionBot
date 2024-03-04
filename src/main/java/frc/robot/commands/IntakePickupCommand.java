package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePickupCommand extends Command {

    private final IntakeSubsystem Intake;

    public IntakePickupCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
        Intake.IntakeInput();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return true;
    }
}