package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Robot;

public class IntakeDeliverCommand extends Command {

    private final IntakeSubsystem Intake;

    public IntakeDeliverCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
    }

    public void execute() {
        Intake.IntakeDeliver();
    }

    public boolean isFinished() {
        if(!Intake.intakeSensor) {
            return true;
        } else {
            return false;
        }
    }
}