// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ArmSetpoint;
import frc.robot.commands.ArmSetpointCommand;
import frc.robot.commands.ClimberControlCommand;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // public boolean intakeSensor, useSensor;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // private DriveSubsystem driveSubsystem;
  // private FlywheelSubsystem flywheelSubsystem;
  // private IntakeSubsystem intakeSubsystem;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  // IntakeSubsystem intakeSubsystem;
  private static final String kAuton1 = "1. Leave front speaker";
  private static final String kAuton2 = "2. Leave Blue Amp Side Speaker";
  private static final String kAuton3 = "3. Leave Red Amp Side Speaker";
  private static final String kAuton4 = "4. Leave Source Side Speaker";
  private static final String kAuton5 = "5. Launch Only";
  private static final String kAuton6 = "6. Forward";

  private String m_autoSelected; // This selects between the two autonomous
  public SendableChooser<String> m_chooser = new SendableChooser<>();
  ArmSetpoint currentSetpoint;
  ArmPivot armPivot = new ArmPivot();
  ClimberSubsystem ClimberControl = new ClimberSubsystem();
  private double POVvalue;
  Limelight m_limelightSubsystem;

  @Override
  public void robotInit() {
    // intakeSubsystem = new IntakeSubsystem();
    // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    // SmartDashboard.putString("Current Auton:", m_autoSelected);
    // if (isReal()) {
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // Pose2d poseA, poseB, poseC;
    // //Logger.recordOutput("MyPose2d", poseA);
    // //Logger.recordOutput("MyPose2dArray", poseA, poseB);
    // //Logger.recordOutput("MyPose2dArray", new Pose2d[] { poseA, poseB });
    // new PowerDistribution(1, ModuleType.kRev);
    // // Enables power distribution logging
    // } else {
    // setUseTiming(false); // Run as fast as possible
    // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
    // AdvantageScope (or prompt the user)
    // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    // "_sim"))); // Save outputs to a new log
    // }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or
    // metadata values may
    // be added.

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // intakeSubsystem = new IntakeSubsystem();
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_robotDrive.m_gyro.reset();
    // driveSubsystem = new DriveSubsystem();
    // flywheelSubsystem = new FlywheelSubsystem();

    m_robotContainer.intakeSubsystem.IntakeStop();
    armPivot.initPivot();
    // flywheelSubsystem.FlywheelStop();

    m_chooser.setDefaultOption("1. Leave Front Speaker", kAuton1);
    m_chooser.addOption("2. Leave Blue Amp Side Speaker", kAuton2);
    m_chooser.addOption("3. Leave Red Amp Side Speaker", kAuton3);
    m_chooser.addOption("4. Leave Source Side Speaker", kAuton4);
    m_chooser.addOption("5. Forward Only", kAuton5);
    m_chooser.addOption("6. Launch Only", kAuton6);
    // prettyLights1.setLEDs(.01);
    m_robotContainer.m_robotDrive.init();

    SmartDashboard.putData(m_chooser); // displays the auton options in shuffleboard, put in init block
    m_limelightSubsystem = new Limelight();
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("ArmEncoder", armPivot.getPivotPosition());
    // SmartDashboard.putNumber("Proximity", Intake.noteSensor.getProximity());

    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("GyroYaw", m_robotContainer.m_robotDrive.m_gyro.getYaw());
    SmartDashboard.putNumber("GyroAngle", m_robotContainer.m_robotDrive.m_gyro.getAngle());
    SmartDashboard.putBoolean("Field Oriented", m_robotContainer.fieldOriented);

    // m_robotContainer.intakeSubsystem.intakeSensor=m_robotContainer.intakeSubsystem.m_noteSensor.get();

    SmartDashboard.putNumber("FlywheelRunning?", m_robotContainer.flywheelSubsystem.runFlywheel);
    // SmartDashboard.putNumber("Flywheel Setpoint", flywheelSubsystem.setpoint);
    // flywheelSubsystem.periodicFlywheel();
    m_limelightSubsystem.updateDashboard();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

    // SmartDashboard.putBoolean("Proximity",
    // m_robotContainer.intakeSubsystem.m_noteSensor.get());
    SmartDashboard.putBoolean("Proximity", m_robotContainer.intakeSubsystem.intakeSensor);

  }


  @Override
  public void autonomousInit() {
    m_robotContainer.m_robotDrive.m_gyro.reset();
    m_autoSelected = m_chooser.getSelected(); // pulls auton option selected from shuffleboard
    SmartDashboard.putString("Current Auton:", m_autoSelected);

    switch (m_autoSelected) {

      case kAuton1:
        m_autonomousCommand = m_robotContainer.getFrontSpeakerLeaveCommand();
        CommandScheduler.getInstance()
            .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
                .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
                .andThen(new WaitCommand(1))
                .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
                .andThen(new WaitCommand(1))
                .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint))
                .andThen(new IntakeInputCommand(m_robotContainer.intakeSubsystem))
                .andThen(m_autonomousCommand)
                .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Six, currentSetpoint))
                .andThen(new WaitCommand(1))
                .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem)));
      default:
        break;
      case kAuton2:
        m_autonomousCommand = m_robotContainer.getBlueAmpSideLeaveCommand();
        CommandScheduler.getInstance()
            .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
                .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
                .andThen(new WaitCommand(.5))
                .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
                .andThen(new WaitCommand(1))
                .andThen(m_autonomousCommand));
        break;
      case kAuton3:
        m_autonomousCommand = m_robotContainer.getRedAmpSideLeaveCommand();
        CommandScheduler.getInstance()
            .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
                .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
                .andThen(new WaitCommand(.5))
                .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
                .andThen(new WaitCommand(1))
                .andThen(m_autonomousCommand));
        break;
      case kAuton4:
        m_autonomousCommand = m_robotContainer.getSourceSideLeaveCommand();
        CommandScheduler.getInstance()
            .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
                .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
                .andThen(new WaitCommand(.5))
                .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
                .andThen(new WaitCommand(.5))
                .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Five, currentSetpoint))
                .andThen(new WaitCommand(.5))
                .andThen(m_autonomousCommand));
        break;
      case kAuton5:
      m_autonomousCommand = m_robotContainer.getGoForwardsCommand();
      CommandScheduler.getInstance()
        .schedule(m_autonomousCommand);
        break;
      case kAuton6:
      CommandScheduler.getInstance()
        .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
        .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
        .andThen(new WaitCommand(.5))
        .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem)));
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.m_robotDrive.m_gyro.reset();
    m_robotContainer.intakeSubsystem.IntakeStop();
    m_robotContainer.flywheelSubsystem.FlywheelStop();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {

    // SmartDashboard.putBoolean("Proximity",
    // m_robotContainer.intakeSubsystem.m_noteSensor.get());
    SmartDashboard.putBoolean("Proximity", m_robotContainer.intakeSubsystem.intakeSensor);
   
    // Driver Controls

    if (m_robotContainer.m_leftDriverController.getRawButtonPressed(kFieldOrientedToggle_LB)) { // Toggles Field Oriented Mode
      if (m_robotContainer.fieldOriented == true) {
        m_robotContainer.fieldOriented = false;
      } else {
        m_robotContainer.fieldOriented = true;
      }
    }
    if (m_robotContainer.m_leftDriverController.getRawButtonPressed(kGyroReset_Start)) { // Resets the Gyro
      m_robotContainer.m_robotDrive.m_gyro.reset();
    }

    if (m_robotContainer.m_leftDriverController.getRawButton(kPrecisionDriving_Trigger)) { //Precise driving mode
      m_robotContainer.speedMultiplier = kSpeedMultiplierPrecise;
    } else {
      m_robotContainer.speedMultiplier = kSpeedMultiplierDefault;
    }

    if (m_robotContainer.m_rightDriverController.getRawButton(kPrecisionDriving_Trigger)) {  //Driver's right trigger lowers arm to go under stage
      CommandScheduler.getInstance()
        .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Seven, currentSetpoint));
    }
        if (m_robotContainer.m_rightDriverController.getRawButtonReleased(kPrecisionDriving_Trigger)){ //returns the arm to transit
      CommandScheduler.getInstance()
      .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint));
    }

    if (m_robotContainer.m_rightDriverController.getRawButtonPressed(3)) {
      CommandScheduler.getInstance()
      .schedule(new LimelightStrafeCommand(m_robotContainer.m_robotDrive, m_limelightSubsystem));
    }
    //Operator Controls

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakePickup_LB) || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakePickup_LB)) { // Runs the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakeStop_Back) || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakeStop_Back)) { // Stops the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kIntakeDrop_RB) || m_robotContainer.m_operatorController2.getRawButtonPressed(kIntakeDrop_RB)) { // Reverses the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeDropCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint1Button_A)) { // Sets the Arm to intake position and runs the intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint))
              .andThen(new IntakePickupCommand(m_robotContainer.intakeSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      currentSetpoint = ArmSetpoint.One;
    }

    if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint1Button_A)) { // Sets the Arm to intake position
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint)));
      currentSetpoint = ArmSetpoint.One;
    }

    if (m_robotContainer.m_operatorController1.getRawButtonPressed(kArmSetpoint4Button_B) || m_robotContainer.m_operatorController2.getRawButtonPressed(kArmSetpoint4Button_B)) { // Transit Position
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Four;

    }
    if (m_robotContainer.m_operatorController1.getPOV() == 0) { // Defense setpoint
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Five, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Five;
    }
    
    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint3Button_X)) { // Goes to amp position and runs the flywheel and intake.
      CommandScheduler.getInstance()
          .schedule(new ArmSetpointCommand(armPivot, ArmSetpoint.Three, currentSetpoint)
              .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
              .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem))
              .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
              
      // .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
      // .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Three;
    }

    if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint3Button_X)) { // Goes to amp position
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Three, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Three;
    }

    if (m_robotContainer.m_operatorController1.getRawButton(kArmSetpoint2Button_Y)) { // Goes to speaker position and runs the flywheel and intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint))
              .andThen(new FlywheelStartCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new WaitCommand(1))
              .andThen(new IntakeDeliverCommand(m_robotContainer.intakeSubsystem))
              .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
              .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem))
              .andThen(new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint)));
      // .andThen(new FlywheelStopCommand(m_robotContainer.flywheelSubsystem))
      // .andThen(new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Four;
    }

    if (m_robotContainer.m_operatorController2.getRawButton(kArmSetpoint2Button_Y)) { // Goes to speaker position
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint)));
    }  
      
    CommandScheduler.getInstance().schedule(
          (new ClimberControlCommand(ClimberControl, Math.abs(MathUtil.applyDeadband(m_robotContainer.m_operatorController1.getRawAxis(1), kClimberDeadband)))));

  
    
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

    //POVvalue = m_robotContainer.m_operatorController1.getPOV();
    //SmartDashboard.putNumber("POV value", POVvalue);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}