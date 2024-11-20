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
        Intake.IntakeRun();
    }

    public void execute() {

    }
    @Override
    public void end(boolean interrupted) {
            Intake.IntakeStop();
            Intake.IntakeRun();
    }

    public boolean isFinished() {
        if (Intake.intakeSensor) {
            Intake.IntakeStop();
            Intake.IntakeRun();
            return true;
        } else {
            return false;
        }
    }
}