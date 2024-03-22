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
        //m_rightFlywheelSparkMax.set(0);
        //m_leftFlywheelSparkMax.set(0);
        flywheelRunning=false;

        // SmartDashboard.putNumber("R P Gain", R_kP);
        // SmartDashboard.putNumber("R I Gain", R_kI);
        // SmartDashboard.putNumber("R D Gain", R_kD);
        // SmartDashboard.putNumber("R I Zone", R_kIz);
        // SmartDashboard.putNumber("R Feed Forward", R_kFF);
        // SmartDashboard.putNumber("L P Gain", L_kP);
        // SmartDashboard.putNumber("L I Gain", L_kI);
        // SmartDashboard.putNumber("L D Gain", L_kD);
        // SmartDashboard.putNumber("L I Zone", L_kIz);
        // SmartDashboard.putNumber("L Feed Forward", L_kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);
        // SmartDashboard.putNumber("Right Flywheel Speed", m_rightFlywheelEncoder.getVelocity());
        // SmartDashboard.putNumber("Left Flywheel Speed", m_leftFlywheelEncoder.getVelocity());
        // SmartDashboard.putNumber("Flywheel Setpoint", MaxRPM);
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
        // System.out.println("Starting Flywheel");
        // double rp = SmartDashboard.getNumber("R P Gain", 0);
        // double ri = SmartDashboard.getNumber("R I Gain", 0);
        // double rd = SmartDashboard.getNumber("R D Gain", 0);
        // double riz = SmartDashboard.getNumber("R I Zone", 0);
        // double rff = SmartDashboard.getNumber("R Feed Forward", 0);
        // double lp = SmartDashboard.getNumber("L P Gain", 0);
        // double li = SmartDashboard.getNumber("L I Gain", 0);
        // double ld = SmartDashboard.getNumber("L D Gain", 0);
        // double liz = SmartDashboard.getNumber("L I Zone", 0);
        // double lff = SmartDashboard.getNumber("L Feed Forward", 0);
        // double max = SmartDashboard.getNumber("Max Output", 0);
        // double min = SmartDashboard.getNumber("Min Output", 0);
        // if((rp != R_kP)) { m_rightFlywheelPIDController.setP(rp); R_kP = rp; }
        // if((ri != R_kI)) { m_rightFlywheelPIDController.setI(ri); R_kI = ri; }
        // if((rd != R_kD)) { m_rightFlywheelPIDController.setD(rd); R_kD = rd; }
        // if((riz != R_kIz)) { m_rightFlywheelPIDController.setIZone(riz); R_kIz = riz; }
        // if((rff != R_kFF)) { m_rightFlywheelPIDController.setFF(rff); R_kFF = rff; }
        // if((lp != L_kP)) { m_rightFlywheelPIDController.setP(lp); L_kP = lp; }
        // if((li != L_kI)) { m_rightFlywheelPIDController.setI(li); L_kI = li; }
        // if((ld != L_kD)) { m_rightFlywheelPIDController.setD(ld); L_kD = ld; }
        // if((liz != L_kIz)) { m_rightFlywheelPIDController.setIZone(liz); L_kIz = riz; }
        // if((lff != L_kFF)) { m_rightFlywheelPIDController.setFF(lff); L_kFF = rff; }
        // if((max != kMaxOutput) || (min != kMinOutput)) { 
        //   m_rightFlywheelPIDController.setOutputRange(min, max);
        //   m_leftFlywheelPIDController.setOutputRange(min, max); 
        //   kMinOutput = min; kMaxOutput = max; 
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
      if((setpoint - m_rightFlywheelEncoder.getVelocity()) < 25) {
        if((setpoint - m_leftFlywheelEncoder.getVelocity()) < 25) {
          return true;
        }
      }
      return false;
    }
}