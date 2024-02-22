package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    CANSparkMax m_rightClimberSparkMax = new CANSparkMax(16, MotorType.kBrushless);
    CANSparkMax m_leftClimberSparkMax = new CANSparkMax(17, MotorType.kBrushless);

    public void ClimberExtend() {
        m_rightClimberSparkMax.set(1);
        m_leftClimberSparkMax.set(1);
    }

    public void ClimberRetract() {
        m_rightClimberSparkMax.set(-1);
        m_leftClimberSparkMax.set(-1);
    }
}
