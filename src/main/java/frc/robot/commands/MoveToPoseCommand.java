package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class MoveToPoseCommand extends Command{
    private double x;
    private double y;
    private double rotation;

    Trajectory k_trajectory;
    SwerveControllerCommand swerveControllerCommand;
    DriveSubsystem m_robotDrive;
    TrajectoryConfig config;
    
    public MoveToPoseCommand(DriveSubsystem m_robotDrive, double x, double y, double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.m_robotDrive = m_robotDrive;
    }

    public void initialize(){

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
     
        config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
            new Rotation2d(0)),List.of(), new Pose2d(x, y,
            new Rotation2d(Math.toRadians(rotation))), config);

        swerveControllerCommand = new SwerveControllerCommand(
            k_trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(k_trajectory.getInitialPose());

        swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public void execute(){
    }

    public boolean isFinished(){
        return true;
    }

    public void end(){  
    }
}