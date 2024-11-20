package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInputCommand extends Command {

    private final IntakeSubsystem Intake;

    public IntakeInputCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
        addRequirements(thisIntake);
    }

    public void initialize() {
        Intake.IntakeInput();
        Intake.IntakeRun();
    }

    public void execute() {

    }

    public boolean isFinished() {

            return true;
        }
    }