// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;


import frc.robot.commands.ArmSetpointCommand;
import frc.robot.commands.ClimberControlCommand;
import frc.robot.commands.DriveStopCommand;
import frc.robot.commands.FlywheelLaunchCommand;
import frc.robot.commands.IntakePickupCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.LimelightStrafeCommand;
import frc.robot.commands.IntakeDeliverCommand;
import frc.robot.commands.IntakeDropCommand;
import frc.robot.commands.IntakeInputCommand;
import frc.robot.commands.FlywheelStartCommand;
import frc.robot.commands.FlywheelStopCommand;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PrettyLights;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private String m_autoSelectedString;
  public SendableChooser<String> m_chooser = new SendableChooser<>();
  ArmSetpoint currentSetpoint;
  ArmPivot armPivot = new ArmPivot();
  ClimberSubsystem ClimberControl = new ClimberSubsystem();
  private double POVvalue;
  Limelight m_limelightSubsystem;
  PrettyLights lights;
  public double timeColor;
  Timer timer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_chooser.setDefaultOption("1. Straight Ahead", OIConstants.kAuton1);
    m_chooser.addOption("2. S Pattern", OIConstants.kAuton2);
    m_chooser.addOption("3. S with a twist", OIConstants.kAuton3);
    m_chooser.addOption("4. Show Off", OIConstants.kAuton4);
    m_chooser.addOption("5. Rotating Fish", OIConstants.kAuton5);
    m_chooser.addOption("6. Maryland Hat", OIConstants.kAuton6);
    m_chooser.addOption("7. Trash", OIConstants.kAuton7);
    m_chooser.addOption("8. Rat", OIConstants.kAuton8);
    m_chooser.addOption("9. G>^v", OIConstants.kAuton9);
   SmartDashboard.putData(m_chooser);
    }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autoSelectedString=m_chooser.getSelected();

    SmartDashboard.putString("Current Auton:", m_autoSelectedString);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoSelectedString);
    /*switch(autoSelected) {
      case "My Auto":
        autonomousCommand = new MyAutoCommand();
        break;
      case "Default Auto":
      default:
        autonomousCommand = new ExampleCommand();
        break;
    }*/

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (timer.getMatchTime() <= 20) {
      timeColor = -.11;
    } else {
      timeColor = .53;
    }
    // SmartDashboard.putBoolean("Proximity",
    // m_robotContainer.intakeSubsystem.m_noteSensor.get());
    SmartDashboard.putBoolean("Proximity", m_robotContainer.intakeSubsystem.intakeSensor);

    // Driver Controls

    if (m_robotContainer.m_leftDriverController.getRawButtonPressed(kFieldOrientedToggle_LB)) { // Toggles Field
                                                                                                // Oriented Mode
      if (m_robotContainer.fieldOriented == true) {
        m_robotContainer.fieldOriented = false;
      } else {
        m_robotContainer.fieldOriented = true;
      }
    }
    if (m_robotContainer.m_leftDriverController.getRawButtonPressed(kGyroReset_Start)) { // Resets the Gyro
      m_robotContainer.m_robotDrive.m_gyro.reset();
    }

    if (m_robotContainer.m_leftDriverController.getRawButton(kPrecisionDriving_Trigger)) { // Precise driving mode
      m_robotContainer.speedMultiplier = kSpeedMultiplierPrecise;
    } else {
      m_robotContainer.speedMultiplier = kSpeedMultiplierDefault;
    }

    if (m_robotContainer.m_rightDriverController.getRawButton(kPrecisionDriving_Trigger)) { // Driver's right trigger
                                                                                            // lowers arm to go under
                                                                                            // stage
      CommandScheduler.getInstance()
          .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Seven, currentSetpoint));
    }
    if (m_robotContainer.m_rightDriverController.getRawButtonReleased(kPrecisionDriving_Trigger)) { // returns the arm
                                                                                                    // to transit
      CommandScheduler.getInstance()
          .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint));
    }

    if (m_robotContainer.m_rightDriverController.getRawButton(3)) { // limelight line up
      CommandScheduler.getInstance()
          .schedule(new LimelightStrafeCommand(m_robotContainer.m_robotDrive, m_limelightSubsystem));
    }
    // Operator Controls

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakePickup_LB)
        // || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakePickup_LB)
        ) { // Runs the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakeStop_Back)
        // || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakeStop_Back)
        ) { // Stops the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakeDrop_RB)
        // || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakeDrop_RB)
        ) { // Reverses the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeDropCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint1Button_A)) { // Sets the Arm to intake position
                                                                                      // and runs the intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint))
              .alongWith(new IntakePickupCommand(m_robotContainer.intakeSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Four;
    }

    // if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint1Button_A)) { // Sets the Arm to intake position
    //   CommandScheduler.getInstance().schedule(
    //       (new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint)));
    //   currentSetpoint = ArmSetpoint.One;
    // }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kArmSetpoint4Button_B)
        // || m_robotContainer.m_operatorController2.getRawButtonPressed(kArmSetpoint4Button_B)
        ) { // Transit Position
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Four;

    }
    if (m_robotContainer.m_operatorController1.getPOV() == 0) { // Defense setpoint
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Five, currentSetpoint)));
      currentSetpoint = ArmSetpoint.One;
    }

    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint3Button_X)) { // Goes to amp position and runs
                                                                                      // the flywheel and intake.
      CommandScheduler.getInstance()
          .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Three, currentSetpoint)
              .alongWith(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
              .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem))
              .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));

      // .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
      // .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Three;
    }

    // if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint3Button_X)) { // Goes to amp position
    //   CommandScheduler.getInstance()
    //       .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Three, currentSetpoint)));
    //   currentSetpoint = ArmSetpoint.Three;
    // }

    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint2Button_Y)) { // Goes to speaker position and
                                                                                      // runs the flywheel and intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
              .alongWith(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
              .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      // .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
      // .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Four;
    }

    // if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint2Button_Y)) { // Goes to speaker position
    //   CommandScheduler.getInstance().schedule(
    //       (new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint)));
    // }

    CommandScheduler.getInstance().schedule(
        (new ClimberControlCommand(ClimberControl,
            Math.abs(MathUtil.applyDeadband(m_robotContainer.m_operatorController1.getRawAxis(1), kClimberDeadband)))));

    // if (m_robotContainer.m_operatorController1.getLeftTriggerAxis() > .5) {
    // //Controls the flywheel
    // if(!m_robotContainer.flywheelSubsystem.flywheelRunning) {
    // CommandScheduler.getInstance()
    // .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));
    // }
    // } else {
    // if(m_robotContainer.flywheelSubsystem.flywheelRunning) {
    // CommandScheduler.getInstance()
    // .schedule((new FlywheelStopCommand(m_robotContainer.flywheelSubsystem)));
    // }
    // }
    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kFlywheelStart_Start)) {
      SmartDashboard.putBoolean("flywheelRunning", m_robotContainer.flywheelSubsystem.flywheelRunning);
      if (!m_robotContainer.flywheelSubsystem.flywheelRunning) {
        CommandScheduler.getInstance()
            .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));

      } else {
        CommandScheduler.getInstance()
            .schedule((new FlywheelStopCommand(m_robotContainer.flywheelSubsystem)));
      }
    }
    if (m_robotContainer.m_operatorController1.getRightTriggerAxis() > .5) { // Runs the intake
      m_robotContainer.intakeSubsystem.m_intakeSparkMax.set(.5);
      // CommandScheduler.getInstance()
      // .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));
      // flywheelSubsystem.flywheelRunning = true;
      // } else {
      // flywheelSubsystem.flywheelRunning = false;
    }
    if (timer.getMatchTime() <= 20) {
      lights.setLEDs(timeColor);
    } else if (m_limelightSubsystem.onTarget) {
      lights.setLEDs(lights.DARK_GREEN);
    } else if (m_robotContainer.intakeSubsystem.intakeSensor) {
      lights.setLEDs(lights.YELLOW);
    } else {
      lights.setLEDs(timeColor);
    }
    // POVvalue = m_robotContainer.m_operatorController1.getPOV();
    // SmartDashboard.putNumber("POV value", POVvalue);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
