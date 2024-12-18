package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPivot;

public class ArmSetpointCommand extends Command {

    private ArmPivot armPivot;
    private ArmConstants.ArmSetpoint goalArmSetpoint;

    public ArmSetpointCommand(ArmPivot armPivot, ArmConstants.ArmSetpoint goalArmSetpoint) {

        this.armPivot = armPivot;
        this.goalArmSetpoint = goalArmSetpoint;
        addRequirements(armPivot);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
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
        if(Math.abs(armPivot.getPivotPosition() - armPivot.pivotSetpoint) < 2)
        {
            return true;
        } else {
            return false;
        }
    }
}
