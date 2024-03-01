package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Robot;

public class IntakeInputCommand extends Command {

    private final IntakeSubsystem Intake;

    public IntakeInputCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
    }

    public void execute() {
        Intake.IntakeRun();
    }

    public boolean isFinished() {
        /*
         * //if ( !Intake.m_noteSensor.get() || Intake.IntakeSpeed <= 0) {
         * if ( Intake.intakeSensor || Intake.IntakeSpeed <= 0) {
         * Intake.IntakeStop();
         * Intake.IntakeRun();
         * return true;
         * } else {
         * return false;
         * }
         */
        return true;
    }
}