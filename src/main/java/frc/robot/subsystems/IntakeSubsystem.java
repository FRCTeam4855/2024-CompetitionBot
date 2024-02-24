package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public final CANSparkMax m_intakeSparkMax;
    // private final SparkPIDController m_intakePIDController;
    public DigitalInput m_noteSensor;
    public double IntakeSpeed = 0;
    public static boolean intakeSensor;
    public boolean useSensor;

    // int proximity = noteSensor.getProximity();
    public IntakeSubsystem() {
        m_intakeSparkMax = new CANSparkMax(10, MotorType.kBrushless);
        // m_intakePIDController = m_intakeSparkMax.getPIDController();
        m_noteSensor = new DigitalInput(0);
    }

    @Override
    public void periodic() {
        intakeSensor=!m_noteSensor.get(); // Inverting the sensor as it reads false when a note is detected
        if(useSensor && intakeSensor)
            IntakeStop();
    }

    public void IntakeInput() {
        m_intakeSparkMax.set(.5);
        IntakeSpeed = .5;
        useSensor = true;
    }

    public void IntakeOutput() {
        m_intakeSparkMax.set(-.5);
        IntakeSpeed = -.5;
        useSensor = false;
    }

    public void FeedNote() {
        useSensor = false;
        m_intakeSparkMax.set(.5);
        IntakeSpeed = .5; 
    }

    public void IntakeStop() {
        m_intakeSparkMax.set(0);
        IntakeSpeed = 0;
        useSensor = false;
    }

    public void IntakeRun() {
        m_intakeSparkMax.set(IntakeSpeed);
        if(!intakeSensor) {
            useSensor=true;
        } else {
            useSensor=false;
        }  

    }
}