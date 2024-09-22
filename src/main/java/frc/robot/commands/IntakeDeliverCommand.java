package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeliverCommand extends Command {

    private final IntakeSubsystem Intake;

    public IntakeDeliverCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
        Intake.IntakeDeliver();
    }

    public void execute() {

    }

    public boolean isFinished() {
        if(!Intake.intakeSensor) {
            Intake.IntakeStop();
            Intake.IntakeRun();
            return true;
        } else {
            return false;
        }
    }
}