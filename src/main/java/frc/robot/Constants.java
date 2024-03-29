// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  // /** Represents a digital button on an XboxController. */
  // public enum Button {
  // kLeftBumper(5),
  // kRightBumper(6),
  // kLeftStick(9),
  // kRightStick(10),
  // kA(1),
  // kB(2),
  // kX(3),
  // kY(4),
  // kBack(7),
  // kStart(8);

  public static final int kLeftDriverControllerPort = 0;  
  public static final int kRightDriverControllerPort = 1;
  public static final double kDriveDeadband = 0.05;
  public static final int kPrecisionDriving_Trigger = 1;
  public static final int kDriverArmOverride_Trigger = 1;
  public static final int kOperatorControllerPort1 = 2;
  public static final int kOperatorControllerPort2 = 3;
  public static final int kArmSetpoint1Button_A = 1;
  public static final int kArmSetpoint4Button_B = 2;
  public static final int kArmSetpoint3Button_X = 3;
  public static final int kArmSetpoint2Button_Y = 4;
  public static final int kFieldOrientedToggle_LB = 5;
  public static final int kGyroReset_Start = 4;
  public static final int kIntakePickup_LB = 5;
  public static final int kIntakeStop_Back = 7;
  public static final int kIntakeDrop_RB = 6;
  public static final int kFlywheelStart_Start = 8;
  public static final double kClimberDeadband = 0.2;
  

  public static final double kArmSetpoint1 = .05; // Intake
  public static final double kArmSetpoint2 = 12; // Speaker
  public static final double kArmSetpoint3 = 100; // Amp
  //public static final double kArmSetpoint4 = 15; // Transit
  public static final double kArmSetpoint4 = 55; // Transit
  public static final double kArmSetpoint5 = 15; // D-D-D-Defense 
  public static final double kArmSetpoint6 = 28; // Long shot
  public static final double kArmSetpoint7 = 15; // Under Stage
  public static final double kArmPivotSlop = 0; // acceptable range for arm extension setpoints
  public static final double kSpeedMultiplierDefault = 1;   // the default speed when no accessory buttons are held down
  public static final double kSpeedMultiplierPrecise = 0.5; // the speed when the trigger is held down for precise movements

  public enum ArmSetpoint {
    One, Two, Three, Four, Five, Six, Seven
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3.0; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 3.14; // flips the motor
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 3.14; // flips the motor
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2; // Zero offset: 0.505

    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearLeftTurningCanId = 4; // Zero offset: 0.09

    public static final int kRearRightDrivingCanId = 5;
    public static final int kRearRightTurningCanId = 6; // Zero offset: 0.445

    public static final int kFrontRightDrivingCanId = 7;
    public static final int kFrontRightTurningCanId = 8; // Zero offset: 0.515

    public static final int kArmPivotRightCanId = 20;
    public static final int kArmPivotLeftCanId = 21;

    public static final int kFlyWheelTopCanId = 11;
    public static final int kFlyWheelBottomCanId = 12;

    public static final int kIntakeCanId = 10;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    // public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = false;
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0889;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 6.122; // previously (45.0 * 22) / (kDrivingMotorPinionTeeth *
                                                               // 15)
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = .1;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1; // -1
    public static final double kTurningMaxOutput = 1; // 1
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3; // 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}