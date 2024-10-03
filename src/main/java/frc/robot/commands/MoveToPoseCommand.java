package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
            Trajectory k_trajectory;
            SwerveControllerCommand swerveControllerCommand;
            DriveSubsystem m_robotDrive;

    public MoveToPoseCommand(DriveSubsystem driveSubsystem, double x, double y, double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        m_robotDrive = driveSubsystem;
    }

    public void initialize(){

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

          /* = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                new Rotation2d(0)),List.of(),new Pose2d(3, 1, new Rotation2d(0)),config);  //This clears a compiler error, but is overwritten later*/
        

        k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
            new Rotation2d(0)),List.of(), new Pose2d(x, y,
            new Rotation2d(Math.toRadians(rotation))), config);
    }

    public void execute(){
    }

    public void end(){
        swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public boolean isFinished(){
        return true;
    }
}
