package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.RobotContainer;

public class IntakePickupCommand extends Command {


    private final IntakeSubsystem Intake;

    public IntakePickupCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
    }

    public void execute() {
        Intake.IntakeInput();
    }

    public boolean isFinished() {
        if ( !Intake.m_noteSensor.get()) {
            Intake.IntakeStop();
            return true;
        } else {
            return false;
        }
    }
}