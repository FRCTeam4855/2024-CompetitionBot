package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.AdjArmFeedforward;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class ArmPivot extends SubsystemBase {
  public double pivotSetpoint;
  CANSparkMax m_armPivot = new CANSparkMax(9, MotorType.kBrushless);
  public SparkPIDController pivotPIDController = m_armPivot.getPIDController();
  SparkAbsoluteEncoder m_pivotEncoder = m_armPivot.getAbsoluteEncoder(Type.kDutyCycle);
  
  double kS = 0.12;
  double kG = 0.495;
  double kV = 0;
  // double kS = .001;
  // double kG = .20;
  // double kV = 5.85;
  // double kP = .0175;
  double kP = 0.03;
  double kI = 0;
  double kD = 0;
  AdjArmFeedforward feedforward = new AdjArmFeedforward(kS, kG, kV);
 
 
  // unnecessary manual controls, not needed when using setpoint control

  public double getPivotPosition() {
       return(m_pivotEncoder.getPosition());
    //return (rawPosition);
    //return ((rawPosition+zeroOffset)%360);
    // if (rawPosition - zeroOffset >= 0)
    //   return (rawPosition - zeroOffset);
    // else
    //   return (rawPosition - zeroOffset + 360);
  }
public void initPivot() {
    // PID coefficients
    m_armPivot.restoreFactoryDefaults();
    m_armPivot.setIdleMode(IdleMode.kBrake);
    // SparkAbsoluteEncoder m_pivotEncoder =
    // m_armPivot.getAbsoluteEncoder(Type.kDutyCycle);
    // double kP = .0175;
    // double kP = .01;
    // double kI = 0;
    // double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;
    pivotPIDController.setFeedbackDevice(m_pivotEncoder);
    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    pivotPIDController.setIZone(kIz);
    pivotPIDController.setFF(kFF);
    pivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
    m_pivotEncoder.setPositionConversionFactor(360);
    m_pivotEncoder.setVelocityConversionFactor(360);
    m_armPivot.setInverted(true);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    pivotPIDController.setPositionPIDWrappingMinInput(0);
    pivotPIDController.setPositionPIDWrappingMaxInput(360);
    m_armPivot.setSmartCurrentLimit(30);

    // SmartDashboard.putNumber("kG", kG);
    // SmartDashboard.putNumber("kV", kV);
    // SmartDashboard.putNumber("FFvalue", 0);
    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    // SmartDashboard.putNumber("kD", kD);
    // SmartDashboard.putNumber("FFvalue", feedforward.calculate(Math.toRadians(8.2), Math.toRadians(1)));
  }

 @Override
  public void periodic() {
   }

  public void setPivotSetpoint(ArmConstants.ArmSetpoint armSetpoint) {
     if (armSetpoint == ArmConstants.ArmSetpoint.One)
      pivotSetpoint = ArmConstants.kArmSetpoint1;
    if (armSetpoint == ArmConstants.ArmSetpoint.Two)
      pivotSetpoint = ArmConstants.kArmSetpoint2;
    if (armSetpoint == ArmConstants.ArmSetpoint.Three)
      pivotSetpoint = ArmConstants.kArmSetpoint3;
    if (armSetpoint == ArmConstants.ArmSetpoint.Four)
      pivotSetpoint = ArmConstants.kArmSetpoint4;
    if (armSetpoint == ArmConstants.ArmSetpoint.Five)
      pivotSetpoint = ArmConstants.kArmSetpoint5;
    if (armSetpoint == ArmConstants.ArmSetpoint.Six)
      pivotSetpoint = ArmConstants.kArmSetpoint6;
    if (armSetpoint == ArmConstants.ArmSetpoint.Seven)
      pivotSetpoint = ArmConstants.kArmSetpoint7;
   }

  public double getPivotSetpointPosition() {
    return pivotSetpoint;
  }

  public boolean isPivotAtSetpoint() {
    return getPivotPosition() - pivotSetpoint <= ArmConstants.kArmPivotSlop;
  }

  public void pivotDaArm() {
    // set PID coefficients
   // kS = SmartDashboard.getNumber("kS", kS);
   // kG = SmartDashboard.getNumber("kG", kG);
   // kV = SmartDashboard.getNumber("kV", kV);
   // kP = SmartDashboard.getNumber("kP", kP);
   // kI = SmartDashboard.getNumber("kI", kI);
   // kD = SmartDashboard.getNumber("kD", kD);
   // SmartDashboard.putNumber("nkS", pivotSetpoint);
    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    feedforward.updateArmFeedforward(kS, kG, kV);
   // SmartDashboard.putNumber("FFvalue", feedforward.calculate(Math.toRadians(pivotSetpoint), Math.toRadians(1)));
    pivotPIDController.setReference(pivotSetpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Math.toRadians(pivotSetpoint), Math.toRadians(1)), ArbFFUnits.kVoltage);
       
  }
}