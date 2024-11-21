package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    private CANSparkMax m_rightFlywheelSparkMax;
    private CANSparkMax m_leftFlywheelSparkMax;
    public double R_kP, R_kI, R_kD, R_kIz, R_kFF, L_kP, L_kI, L_kD, L_kIz, L_kFF,kMaxOutput, kMinOutput, MaxRPM;
    SparkPIDController m_rightFlywheelPIDController;
    SparkPIDController m_leftFlywheelPIDController;
    private RelativeEncoder m_rightFlywheelEncoder;
    private RelativeEncoder m_leftFlywheelEncoder;
    public int runFlywheel;
    public double setpoint;
    public boolean flywheelRunning;
    public boolean flywheelAtSetpoint;
    public double leftFlywheelSpeed;
    public double rightFlywheelSpeed;

    public FlywheelSubsystem() {
        m_rightFlywheelSparkMax = new CANSparkMax(12, MotorType.kBrushless);
        m_leftFlywheelSparkMax = new CANSparkMax(11, MotorType.kBrushless);
        m_rightFlywheelSparkMax.restoreFactoryDefaults();
        m_leftFlywheelSparkMax.restoreFactoryDefaults();
        m_rightFlywheelSparkMax.setIdleMode(IdleMode.kCoast);
        m_leftFlywheelSparkMax.setIdleMode(IdleMode.kCoast);
        m_rightFlywheelPIDController = m_rightFlywheelSparkMax.getPIDController();
        m_leftFlywheelPIDController = m_leftFlywheelSparkMax.getPIDController();
        m_rightFlywheelEncoder = m_rightFlywheelSparkMax.getEncoder();
        m_leftFlywheelEncoder = m_leftFlywheelSparkMax.getEncoder();
        R_kP = 0.0002; 
        R_kI = 0;
        R_kD = 0; 
        R_kIz = 0; 
        R_kFF = 0.000180;
        L_kP = 0.0002; 
        L_kI = 0;
        L_kD = 0; 
        L_kIz = 0; 
        L_kFF = 0.00019;
        //kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = 0;
        MaxRPM = 2500;
        m_rightFlywheelPIDController.setP(R_kP);
        m_rightFlywheelPIDController.setI(R_kI);
        m_rightFlywheelPIDController.setD(R_kD);
        m_rightFlywheelPIDController.setIZone(R_kIz);
        m_rightFlywheelPIDController.setFF(R_kFF);
        m_rightFlywheelPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_rightFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
        m_leftFlywheelPIDController.setP(L_kP);
        m_leftFlywheelPIDController.setI(L_kI);
        m_leftFlywheelPIDController.setD(L_kD);
        m_leftFlywheelPIDController.setIZone(L_kIz);
        m_leftFlywheelPIDController.setFF(L_kFF);
        m_leftFlywheelPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_leftFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
        flywheelRunning=false;
    }

    @Override
    public void periodic() {
      rightFlywheelSpeed = m_rightFlywheelEncoder.getVelocity();
      leftFlywheelSpeed = m_leftFlywheelEncoder.getVelocity();
      SmartDashboard.putNumber("Right Flywheel Speed", rightFlywheelSpeed);
      SmartDashboard.putNumber("Left Flywheel Speed", leftFlywheelSpeed);
      if (rightFlywheelSpeed >= setpoint - 100 && leftFlywheelSpeed >= setpoint - 100) {
        flywheelAtSetpoint = true;
      } else {
        flywheelAtSetpoint = false;
      }
      SmartDashboard.putBoolean("Flywheel At Setpoint", flywheelAtSetpoint);
    }

    public void FlywheelStart(){
      System.out.println("Entering FlywheelStart");
      if(!flywheelRunning) {
        }
        setpoint = SmartDashboard.getNumber("Flywheel Setpoint", MaxRPM);
        m_rightFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        m_leftFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        flywheelRunning=true;
      }
    

    public void FlywheelStop(){
      System.out.println("Entering Flywheel Stop");
      System.out.println("Stopping Flywheel");
      m_rightFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      m_leftFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      flywheelRunning=false;
    }
    
    public void FlywheelLaunch(){
      m_rightFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      m_leftFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      flywheelRunning=true;
    }

    public boolean FlywheelCheck(){
      if((setpoint - m_rightFlywheelEncoder.getVelocity()) < 25) {
        if((setpoint - m_leftFlywheelEncoder.getVelocity()) < 25) {
          return true;
        }
      }
      return false;
    }
}