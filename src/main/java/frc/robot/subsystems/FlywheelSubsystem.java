package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    CANSparkMax m_rightFlywheelSparkMax = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax m_leftFlywheelSparkMax = new CANSparkMax(11, MotorType.kBrushless); // right .4 and left .5 for speaker
    // double kS = 0.12;
    // double kG = 0.495;
    // double kV = 0;
    // double kP = 0.03; 
    // double kI = 0; 
    // double kD = 0; 

    public void FlywheelVariable(int speed) {
        m_rightFlywheelSparkMax.set(speed);
        m_leftFlywheelSparkMax.set(speed);
    }

    public void FlywheelStop() {
        m_rightFlywheelSparkMax.set(0);
        m_leftFlywheelSparkMax.set(0);
    }
}