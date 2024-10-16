package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants.ArmSetpoint;
import frc.robot.Robot;

public class SeqAuto extends SequentialCommandGroup {
    private static DriveSubsystem m_robotDrive;
    private static ArmPivot m_armPivot;
    private static FlywheelSubsystem m_flyWheel;
    private static IntakeSubsystem m_intake;

public SeqAuto(DriveSubsystem driveSubsystem, ArmPivot armPivot, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem){
    
    addRequirements(driveSubsystem, armPivot, flywheelSubsystem, intakeSubsystem);

    m_robotDrive = driveSubsystem;
    m_armPivot = armPivot;
    m_flyWheel = flywheelSubsystem;
    m_intake = intakeSubsystem;

    addCommands(
    /*    new ParallelCommandGroup( 
            new ArmSetpointCommand(m_armPivot, ArmSetpoint.Two),
            new FlywheelStartCommand(m_flyWheel)
            ),
        new IntakeDeliverCommand(m_intake),
        new FlywheelStopCommand(m_flyWheel),
        new ParallelCommandGroup(
            new ArmSetpointCommand(m_armPivot, ArmSetpoint.One),
            //new IntakeInputCommand(m_intake),
            new MoveToPoseCommand(m_robotDrive, 1.5, 0, 0, true)
            ),
        new MoveToPoseCommand(m_robotDrive, 0, 1.5, 0, true)
    );
    */
    new WaitCommand(1),
    new ParallelRaceGroup(new WaitCommand(2),
    new MoveToPoseCommand(m_robotDrive, 1, 0, 0, false)),
   // new WaitCommand(2),
    new ArmSetpointCommand(m_armPivot, ArmSetpoint.Two),
    new MoveToPoseCommand(m_robotDrive, 0, 1, 0, true));

}

    
}
