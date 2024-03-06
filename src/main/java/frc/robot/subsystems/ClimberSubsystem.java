package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ClimberSubsystem extends SubsystemBase {

private CANSparkMax m_ClimberSparkMax;

    public ClimberSubsystem() {
       m_ClimberSparkMax = new CANSparkMax(13, MotorType.kBrushless); 
    
       m_ClimberSparkMax.setIdleMode(IdleMode.kBrake);
    }
    
    public void ClimberControl(double ClimberVoltage) {
       m_ClimberSparkMax.set(ClimberVoltage);
    }   
}
