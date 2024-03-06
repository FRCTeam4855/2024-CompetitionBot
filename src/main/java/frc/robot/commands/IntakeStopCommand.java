package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends Command {
    private IntakeSubsystem Intake;

    public IntakeStopCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {

    }

    public void execute() {
        Intake.IntakeStop();
        Intake.IntakeRun();
    }

    public boolean isFinished() {
        return true;
    }
}