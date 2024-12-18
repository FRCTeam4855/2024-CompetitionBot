// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.ArmConstants.ArmSetpoint;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.ArmSetpointCommand;
import frc.robot.commands.ClimberControlCommand;
import frc.robot.commands.IntakeDropCommand;
import frc.robot.commands.IntakePickupCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.IntakeInputCommand;
import frc.robot.commands.IntakeDeliverCommand;
import frc.robot.commands.LimelightStrafeCommand;
import frc.robot.commands.FlywheelStartCommand;
import frc.robot.commands.FlywheelStopCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final ArmPivot m_armPivot = new ArmPivot();
    private final FlywheelSubsystem m_flyWheel = new FlywheelSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
   

    // The driver's controller
    Joystick m_leftDriverController = new Joystick(OIConstants.kLeftDriverControllerPort);
    Joystick m_rightDriverController = new Joystick(OIConstants.kRightDriverControllerPort);
    // The Operator Controller
    CommandXboxController m_operatorController1 = new CommandXboxController(OIConstants.kOperatorControllerPort1);
    //XboxController m_operatorController2 = new XboxController(OIConstants.kOperatorControllerPort2);

    public static boolean fieldOriented = false;
    public double speedMultiplier = OIConstants.kSpeedMultiplierDefault;
    private final SendableChooser<Command> autoChooser;

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        //Register Named Commands
        NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ArmSetpointCommand(m_armPivot, ArmConstants.ArmSetpoint.One), 
                new IntakePickupCommand(m_intake))));
        NamedCommands.registerCommand("Launch", new SequentialCommandGroup(
            new ParallelCommandGroup(
                new FlywheelStartCommand(m_flyWheel), 
                new ArmSetpointCommand(m_armPivot, ArmConstants.ArmSetpoint.Two)),
            new IntakeDeliverCommand(m_intake), 
            new FlywheelStopCommand(m_flyWheel)));
        NamedCommands.registerCommand("ArmToTransit", new ArmSetpointCommand(m_armPivot, ArmConstants.ArmSetpoint.Four));
        
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
                fieldOriented, true),
            m_robotDrive));

            autoChooser = AutoBuilder.buildAutoChooser(); 
            SmartDashboard.putData("Auto Chooser", autoChooser);             
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
    
////    Driver Controls

       new JoystickButton(m_leftDriverController,OIConstants.kJS_BB)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        new JoystickButton(m_rightDriverController,OIConstants.kJS_LB)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.strafeLeft(),
                m_robotDrive));

        new JoystickButton(m_rightDriverController,OIConstants.kJS_RB)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.strafeRight(),
                m_robotDrive));
       
        new JoystickButton(m_leftDriverController, OIConstants.kJS_RB).debounce(0.1)  //Gyro reset
            .whileTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive)); 
        
        new JoystickButton(m_rightDriverController, OIConstants.kJS_BB) // limelight line up
            .whileTrue(new LimelightStrafeCommand(m_robotDrive, m_limelight));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_LB)  //Field oriented toggle
            .whileTrue(new InstantCommand(
                () -> toggleFieldOriented()));
        
        new JoystickButton(m_leftDriverController, OIConstants.kJS_Trigger)  //Precise Driving Mode set
            .whileTrue(new InstantCommand(
                () -> speedMultiplier=OIConstants.kSpeedMultiplierPrecise));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_Trigger)  //Precise Driving Mode clear
            .whileFalse(new InstantCommand(
                () -> speedMultiplier=OIConstants.kSpeedMultiplierDefault));

        new JoystickButton(m_rightDriverController, OIConstants.kJS_Trigger) //Lower Arm to go under stage    
            .whileTrue(new ArmSetpointCommand(m_armPivot,ArmSetpoint.Seven));

        new JoystickButton(m_rightDriverController, OIConstants.kJS_Trigger) //Return arm to transit position
            .onFalse(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Four));

////    Operator Controls 
 
        m_operatorController1.leftBumper().onTrue(new IntakeInputCommand(m_intake));    //Starts the intake
        m_operatorController1.back().onTrue(new IntakeStopCommand(m_intake));   //Stops the intake     
        m_operatorController1.rightBumper().onTrue(new IntakeDropCommand(m_intake));    //Runs the intake in reverse
        m_operatorController1.start().onTrue(new FlywheelStartCommand(m_flyWheel));     //Starts the Flywheel
        m_operatorController1.leftStick().onTrue(new FlywheelStopCommand(m_flyWheel));  //Stops the Flywheel
        m_operatorController1.axisGreaterThan(1, 0.5)
            .whileTrue(new ClimberControlCommand(m_climberSubsystem,.7))
            .onFalse(new ClimberControlCommand(m_climberSubsystem, 0));  
       m_operatorController1.a()   //Lowers arm, starts intake, raises arm when sensor triggered
            .onTrue(new ArmSetpointCommand(m_armPivot, ArmSetpoint.One)
            .alongWith(new IntakePickupCommand(m_intake))
            .andThen(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Four)));
        m_operatorController1.b().onTrue(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Four)); //Returns the arm to transit position
        m_operatorController1.x().onTrue(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Three)  //Launch sequence for Amp
                .alongWith(new FlywheelStartCommand(m_flyWheel))
                .andThen(new IntakeDeliverCommand(m_intake))
                .andThen(new IntakeStopCommand(m_intake))
                .andThen(new FlywheelStopCommand(m_flyWheel))
                .andThen(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Four)));
        m_operatorController1.y().onTrue(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Two)    //Launch sequence for Speaker
                    .alongWith(new FlywheelStartCommand(m_flyWheel))
                    .andThen(new IntakeDeliverCommand(m_intake))
                    .andThen(new FlywheelStopCommand(m_flyWheel))
                    .andThen(new IntakeStopCommand(m_intake))                
                    .andThen(new ArmSetpointCommand(m_armPivot, ArmSetpoint.Four)));

          }
            /* TODO     
             * Auto Chooser
             * Build paths and Autos
             * Integrate LEDs to commands
             * Investigate loop overruns
             * Remove dead code blocks IN PROGRESS
             * Find a cleaner way to declare these buttons and commands DONE
             * Look at switch statements for defining lists of things (buttons, autons, setpoints, etc.)
             * Program autons
             * Autons with pathweaver
             * Design a new drivestation that fits this laptop and the joysticks
             * 
             * General Notes:
             * I think we're using commands for too simple of tasks. I think we can handle individual things 
             * (toggling states of things especially) with instant commands instead. I think the commands should be used when
             * we're grouping things together to keep the buik of the logic out of this file and in the command files.
             * Also learned that our smart dashboard calls should go in the periodic section of the subsystem.
             */


    private void toggleFieldOriented () {
        fieldOriented = !fieldOriented;
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();



    }
}
