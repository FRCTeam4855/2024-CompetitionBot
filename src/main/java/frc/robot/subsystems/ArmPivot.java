package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


//import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.AdjArmFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSetpoint;
import frc.robot.Constants.ModuleConstants;
import  frc.robot.AdjArmFeedforward;
import static frc.robot.Constants.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class ArmPivot extends SubsystemBase {
  double pivotSetpoint;
  CANSparkMax m_armPivotOne = new CANSparkMax(9, MotorType.kBrushless);
  SparkPIDController pivotPIDController = m_armPivotOne.getPIDController();
  
  double kS = 0.12;
  double kG = 0.495;
  double kV = 0;
  //double kS = .001;
  //double kG = .20;
  //double kV = 5.85;
  //double kP = .0175; 
  double kP = 0.03; 
  double kI = 0; 
  double kD = 0; 
  AdjArmFeedforward feedforward = new AdjArmFeedforward(kS, kG, kV);
  //ArmFeedforward feedforward = new ArmFeedforward(0.001, 0.17, 5.85, .02);
  //ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

  //unnecessary manual controls, not needed when using setpoint control
  
  // public void setPivotDirectionForward() {
  //   m_armPivotOne.set(-1);
  // }

  // public void setPivotDirectionBackward() {
  //   m_armPivotOne.set(1);
  // }

  // public void setPivotStop() {
  //   m_armPivotOne.set(0);
  // }

  // public void armPivotVariable(double speed) {
  //   m_armPivotOne.set(speed);
  // }

  public double getPivotPostion() {
    return m_armPivotOne.getEncoder().getPosition();
  }

  //not needed for absolute encoder
  // public void resetPivotEncoderZero() {
  //   m_armPivotOne.getEncoder().setPosition(0);
  // }



  public void initPivot() {
    // PID coefficients
    m_armPivotOne.restoreFactoryDefaults();
    m_armPivotOne.setIdleMode(IdleMode.kBrake);
    SparkAbsoluteEncoder m_pivotEncoder = m_armPivotOne.getAbsoluteEncoder(Type.kDutyCycle);
    //double kP = .0175; 
    //double kP = .01; 
    //double kI = 0; 
    //double kD = 0; 
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
    m_armPivotOne.setInverted(true);
    pivotPIDController.setPositionPIDWrappingEnabled(true);
    pivotPIDController.setPositionPIDWrappingMinInput(0);
    pivotPIDController.setPositionPIDWrappingMaxInput(360);
    m_armPivotOne.setSmartCurrentLimit(40);


    SmartDashboard.putNumber("kG", kG);
    SmartDashboard.putNumber("kV", kV);
    SmartDashboard.putNumber("FFvalue", 0);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    SmartDashboard.putNumber("FFvalue", feedforward.calculate(Math.toRadians(8.2),Math.toRadians(1)));
  }
    public void setPivotSetpoint(ArmSetpoint armSetpoint) {
    if (armSetpoint == ArmSetpoint.One)
      pivotSetpoint = kArmSetpoint1;
    if (armSetpoint == ArmSetpoint.Two)
      pivotSetpoint = kArmSetpoint2;
    if (armSetpoint == ArmSetpoint.Three)
      pivotSetpoint = kArmSetpoint3;
    if (armSetpoint == ArmSetpoint.Four)
      pivotSetpoint = kArmSetpoint4;
    if (armSetpoint == ArmSetpoint.Five)
      pivotSetpoint = kArmSetpoint5;
  }

  public double getPivotSetpointPosition() {
    return pivotSetpoint;
  }

  public boolean isPivotAtSetpoint() {
    return getPivotPostion() - pivotSetpoint <= kArmPivotSlop;
  }

  public void pivotDaArm() {
    // set PID coefficients
    SmartDashboard.getNumber("kS", kS);
    SmartDashboard.getNumber("kG", kG);
    SmartDashboard.getNumber("kV", kV);
    SmartDashboard.getNumber("kP", kP);
    SmartDashboard.getNumber("kI", kI);
    SmartDashboard.getNumber("kD", kD);
    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    feedforward.updateArmFeedforward(kS, kG, kV);
    SmartDashboard.putNumber("FFvalue", feedforward.calculate(Math.toRadians(pivotSetpoint),Math.toRadians(112)));
    pivotPIDController.setFF(feedforward.calculate(Math.toRadians(pivotSetpoint),Math.toRadians(112)));
    pivotPIDController.setReference(pivotSetpoint, CANSparkMax.ControlType.kPosition);
    kS=SmartDashboard.getNumber("kS", kS);
    kG=SmartDashboard.getNumber("kG", kG);
    kV=SmartDashboard.getNumber("kV", kV);
    kP=SmartDashboard.getNumber("kP", kP);
    kI=SmartDashboard.getNumber("kI", kI);
    kD=SmartDashboard.getNumber("kD", kD);
    SmartDashboard.putNumber("nkS", pivotSetpoint);
    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    feedforward.updateArmFeedforward(kS, kG, kV);
    SmartDashboard.putNumber("FFvalue", feedforward.calculate(Math.toRadians(pivotSetpoint),Math.toRadians(1)));
    //pivotPIDController.setFF(feedforward.calculate(Math.toRadians(pivotSetpoint),Math.toRadians(1)));
    pivotPIDController.setReference(pivotSetpoint, CANSparkMax.ControlType.kPosition, 0, feedforward.calculate(Math.toRadians(pivotSetpoint),Math.toRadians(1)), ArbFFUnits.kVoltage);
    // SmartDashboard.putNumber("PivotSetPoint", pivotSetpoint);
    // SmartDashboard.putNumber("PivotVariable", getPivotPostion());

  }
}

// public class ArmPivot extends SubsystemBase {
//   final SparkAbsoluteEncoder m_ArmEncoder;
//   final CANSparkMax m_ArmSparkMax;
//   final SparkPIDController m_ArmPIDController; 
//   {

//   m_ArmSparkMax = new CANSparkMax(9, MotorType.kBrushless);
//   m_ArmSparkMax.setInverted(true);
//   m_ArmEncoder = m_ArmSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
//   m_ArmEncoder.setVelocityConversionFactor(365);
//   m_ArmEncoder.setPositionConversionFactor(365);
  
//   m_ArmPIDController = m_ArmSparkMax.getPIDController();
//   m_ArmPIDController.setFeedbackDevice(m_ArmEncoder);

//   m_ArmPIDController.setP(.05);
//   m_ArmPIDController.setI(0);
//   m_ArmPIDController.setD(0);
//   m_ArmPIDController.setFF(0);

//   m_ArmPIDController.setReference(20,CANSparkMax.ControlType.kPosition);
//    };  
// }