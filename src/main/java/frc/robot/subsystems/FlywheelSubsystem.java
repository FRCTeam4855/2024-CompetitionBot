package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.FlywheelStartCommand;
import frc.robot.commands.FlywheelStopCommand;
import frc.robot.commands.FlywheelLaunchCommand;
import frc.robot.RobotContainer;

public class FlywheelSubsystem extends SubsystemBase {

    private CANSparkMax m_rightFlywheelSparkMax;
    private CANSparkMax m_leftFlywheelSparkMax;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, MaxRPM;
    SparkPIDController m_rightFlywheelPIDController;
    SparkPIDController m_leftFlywheelPIDController;
    private RelativeEncoder m_rightFlywheelEncoder;
    private RelativeEncoder m_leftFlywheelEncoder;
    public int runFlywheel;
    public double setpoint;
    public boolean flywheelRunning;

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
        kP = 0; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.00017;
        //kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = 0;
        MaxRPM = 2500;
        m_rightFlywheelPIDController.setP(kP);
        m_rightFlywheelPIDController.setI(kI);
        m_rightFlywheelPIDController.setD(kD);
        m_rightFlywheelPIDController.setIZone(kIz);
        m_rightFlywheelPIDController.setFF(kFF);
        m_rightFlywheelPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_rightFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
        m_leftFlywheelPIDController.setP(kP);
        m_leftFlywheelPIDController.setI(kI);
        m_leftFlywheelPIDController.setD(kD);
        m_leftFlywheelPIDController.setIZone(kIz);
        m_leftFlywheelPIDController.setFF(kFF);
        m_leftFlywheelPIDController.setOutputRange(kMinOutput, kMaxOutput);
        m_leftFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
        //m_rightFlywheelSparkMax.set(0);
        //m_leftFlywheelSparkMax.set(0);
        flywheelRunning=false;

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Right Flywheel Speed", m_rightFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Left Flywheel Speed", m_leftFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel Setpoint", MaxRPM);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Right Flywheel Speed", m_rightFlywheelEncoder.getVelocity());
      SmartDashboard.putNumber("Left Flywheel Speed", m_leftFlywheelEncoder.getVelocity());
    }

    public void FlywheelStart(){
      System.out.println("Entering FlywheelStart");
      if(!flywheelRunning) {
        System.out.println("Starting Flywheel");
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        if((p != kP)) { m_rightFlywheelPIDController.setP(p); m_leftFlywheelPIDController.setP(p); kP = p; }
        if((i != kI)) { m_rightFlywheelPIDController.setI(i); m_leftFlywheelPIDController.setI(i); kI = i; }
        if((d != kD)) { m_rightFlywheelPIDController.setD(d); m_leftFlywheelPIDController.setD(d); kD = d; }
        if((iz != kIz)) { m_rightFlywheelPIDController.setIZone(iz); m_leftFlywheelPIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_rightFlywheelPIDController.setFF(ff); m_leftFlywheelPIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_rightFlywheelPIDController.setOutputRange(min, max);
          m_leftFlywheelPIDController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
          /*if((p != kP)) { m_leftFlywheelPIDController.setP(p); kP = p; }
          if((i != kI)) { m_leftFlywheelPIDController.setI(i); kI = i; }
          if((d != kD)) { m_leftFlywheelPIDController.setD(d); kD = d; }
          if((iz != kIz)) { m_leftFlywheelPIDController.setIZone(iz); kIz = iz; }
          if((ff != kFF)) { m_leftFlywheelPIDController.setFF(ff); kFF = ff; }
          if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_leftFlywheelPIDController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
          }  */
        setpoint = SmartDashboard.getNumber("Flywheel Setpoint", MaxRPM);
        m_rightFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        m_leftFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        //m_rightFlywheelSparkMax.set(.5);
        //m_leftFlywheelSparkMax.set(.5);
        flywheelRunning=true;
      }
    }

    public void FlywheelStop(){
      System.out.println("Entering Flywheel Stop");
      System.out.println("Stopping Flywheel");
      m_rightFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      m_leftFlywheelPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      //m_rightFlywheelSparkMax.set(0);
      //m_leftFlywheelSparkMax.set(0);
      flywheelRunning=false;
    }
    
    public void FlywheelLaunch(){
      m_rightFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      m_leftFlywheelPIDController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      flywheelRunning=true;
      //m_rightFlywheelSparkMax.set(.5);
      //m_leftFlywheelSparkMax.set(.5);
      //CommandScheduler.getInstance()
      // .schedule((new FlywheelCheckCommand(m_robotContainer.flywheelSubsystem))); 
      //new WaitUntilCommand(FlywheelCheckCommand());
    }

    public boolean FlywheelCheck(){
      if(Math.abs(setpoint - m_rightFlywheelEncoder.getVelocity()) < 50) {
        if(Math.abs(setpoint - m_leftFlywheelEncoder.getVelocity()) < 50) {
          return true;
        }
      }
      return false;
    }
}