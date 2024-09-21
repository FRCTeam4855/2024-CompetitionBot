// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.LimelightStrafeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final Limelight m_limelight = new Limelight();

    // The driver's controller
    Joystick m_leftDriverController = new Joystick(OIConstants.kLeftDriverControllerPort);
    Joystick m_rightDriverController = new Joystick(OIConstants.kRightDriverControllerPort);
    // The Operator Controller
    XboxController m_operatorController1 = new XboxController(OIConstants.kOperatorControllerPort1);
    XboxController m_operatorController2 = new XboxController(OIConstants.kOperatorControllerPort2);

    public boolean fieldOriented = false;
    public double speedMultiplier = OIConstants.kSpeedMultiplierDefault;

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
        // The left Joystick controls translation of the robot.
        // The right Joystick controls rotation of the robot.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(1) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rightDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband) * OIConstants.kRotateScale,
                true, true),
            m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        /*new JoystickButton(m_driverController, Button.kX.value)
            .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));*/

        new JoystickButton(m_leftDriverController, OIConstants.kGyroReset_Start)  //4855
            .onTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

        new JoystickButton(m_rightDriverController, 3) //4855 0
            .whileTrue(new RunCommand(
                () -> LimelightStrafeCommand.LimelightStrafeCommand(m_robotDrive, m_limelight),
                m_robotDrive, m_limelight)); //4855
  
        new JoystickButton(m_driverController, Button.kRightBumper.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.strafeRight(),
                m_robotDrive)); //4855
             
        new JoystickButton(m_driverController, Button.kBack.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.rotateLeft(),
                m_robotDrive)); //4855

        new JoystickButton(m_driverController, Button.kStart.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.rotateRight(),
                m_robotDrive)); //4855
    } 

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String routineString) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory k_trajectory;  /* = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                new Rotation2d(0)),List.of(),new Pose2d(3, 1, new Rotation2d(0)),config);  //This clears a compiler error, but is overwritten later*/
        SwerveControllerCommand swerveControllerCommand; /* = new SwerveControllerCommand(k_trajectory,
                m_robotDrive::getPose,
                // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);  //This clears a compiler error, but is overwritten later*/

        switch (routineString){
            case OIConstants.kAuton1:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                    new Rotation2d(0)),List.of(),new Pose2d(3, 1, new Rotation2d(0)),config);
            break;

            case OIConstants.kAuton2:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                    new Rotation2d(0)),List.of(new Translation2d(1, 2), new Translation2d(2, 0)),
                    new Pose2d(3, 1, new Rotation2d(0)),config);
            break;

            case OIConstants.kAuton3:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                    new Rotation2d(0)),List.of(new Translation2d(1, 2), new Translation2d(2, 0)),
                    new Pose2d(3, 1, new Rotation2d(Math.toRadians(90))),config);
            break;

            case OIConstants.kAuton4:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                    new Rotation2d(0)),List.of(),new Pose2d(1, 2, new Rotation2d(Math.toRadians(90))),config);
            break;

            case OIConstants.kAuton5:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 1,
                    new Rotation2d(0)),List.of(new Translation2d(-2,0)), new Pose2d(4, 1,
                    new Rotation2d(Math.toRadians(180))),config);
            break;

            case OIConstants.kAuton6:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 1,
                    new Rotation2d(0)),List.of(new Translation2d(1.5,.5)), new Pose2d(2.75, 1.4,
                    new Rotation2d(Math.toRadians(0))),config);
            break;

            case OIConstants.kAuton7:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
                    new Rotation2d(0)),List.of(new Translation2d(-1,0 )), new Pose2d(0, -2,
                    new Rotation2d(Math.toRadians(-45))),config);
            break;

            case OIConstants.kAuton8:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
                    new Rotation2d(0)),List.of(new Translation2d(-2,0 )), new Pose2d(-3, -1,
                    new Rotation2d(Math.toRadians(-135))),config);
            break;

            case OIConstants.kAuton9:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
                    new Rotation2d(0)),List.of(new Translation2d(-2,0 ),new Translation2d(-3, -1.25)), new Pose2d(-1, -3,
                    new Rotation2d(Math.toRadians(-45))),config);
            break;

            default:
                k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
                    new Rotation2d(0)),List.of(), new Pose2d(0, 0,
                    new Rotation2d(Math.toRadians(0))),config);
        }

            /*k_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(1, 2,
                new Rotation2d(0)),List.of(),new Pose2d(2, 0, new Rotation2d(0)),config);
            

            // An example trajectory to follow. All units in meters.
            //Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            //  // Start at the origin facing the +X direction
            //  /*new Pose2d(0, 0, new Rotation2d(0)),
            //  // Pass through these two interior waypoints, making an 's' curve path
            //  List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            //  // End 3 meters straight ahead of where we started, facing forward
            //  new Pose2d(3, 0, new Rotation2d(0)),*/ //4855
            //  new Pose2d(0, 1, new Rotation2d(0)),
            //  // Pass through these two interior waypoints, making an 's' curve path
            //  List.of(new Translation2d(1, 2), new Translation2d(2, 0)),
            //  // End 3 meters straight ahead of where we started, facing forward
            //  new Pose2d(3, 1, new Rotation2d(Math.PI/2)),
            //  config);

    
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

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
