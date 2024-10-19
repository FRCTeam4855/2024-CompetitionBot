package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants.ArmSetpoint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class SeqAuto extends SequentialCommandGroup {
    private static DriveSubsystem m_robotDrive;
    private static ArmPivot m_armPivot;
    private static FlywheelSubsystem m_flyWheel;
    private static IntakeSubsystem m_intake;

    Trajectory k_trajectory;
    Trajectory k_trajectory_2;
    SwerveControllerCommand swerveControllerCommand;
    SwerveControllerCommand swerveControllerCommand2;
    TrajectoryConfig config;
    Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d pose1 = new Pose2d(1 , 0, new Rotation2d(0));
    Pose2d pose2 = new Pose2d(0 , 1, new Rotation2d(0));
public SeqAuto(DriveSubsystem driveSubsystem, ArmPivot armPivot, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem){
    
    addRequirements(driveSubsystem, armPivot, flywheelSubsystem, intakeSubsystem);

    m_robotDrive = driveSubsystem;
    m_armPivot = armPivot;
    m_flyWheel = flywheelSubsystem;
    m_intake = intakeSubsystem;

      var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
      var xController = new PIDController(AutoConstants.kPXController, 0, 0);
      var yController = new PIDController(AutoConstants.kPYController, 0, 0);
      
     
        config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        k_trajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), pose1, config);
        
        k_trajectory_2 = TrajectoryGenerator.generateTrajectory(pose1, List.of(), pose2, config);

        swerveControllerCommand = new SwerveControllerCommand(
            k_trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            xController, yController,thetaController,
            m_robotDrive::setModuleStates, // Drive the robot
            m_robotDrive);
            
        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(k_trajectory.getInitialPose());
   
        swerveControllerCommand2 = new SwerveControllerCommand(
            k_trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            xController, yController,thetaController,
            m_robotDrive::setModuleStates, // Drive the robot
            m_robotDrive);
    
    
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
    
    swerveControllerCommand,
    new ArmSetpointCommand(m_armPivot, ArmSetpoint.Two),
    swerveControllerCommand2
    );

}

    
}
