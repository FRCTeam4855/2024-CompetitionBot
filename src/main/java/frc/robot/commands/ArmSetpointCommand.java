package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmSetpoint;
import frc.robot.subsystems.ArmPivot;

public class ArmSetpointCommand extends Command {

    private ArmPivot armPivot;
    private ArmSetpoint startArmSetpoint;
    private ArmSetpoint goalArmSetpoint;

    public ArmSetpointCommand(ArmPivot armPivot, ArmSetpoint goalArmSetpoint,
            ArmSetpoint startArmSetpoint) {

        this.armPivot = armPivot;
        this.startArmSetpoint = startArmSetpoint;
        this.goalArmSetpoint = goalArmSetpoint;
        addRequirements(armPivot);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armPivot.setPivotSetpoint(goalArmSetpoint);
        armPivot.pivotDaArm();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
