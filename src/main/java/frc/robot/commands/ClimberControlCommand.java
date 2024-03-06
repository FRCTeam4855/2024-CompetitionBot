package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberControlCommand extends Command {
    
    private double ClimberPower;
    private final ClimberSubsystem m_Climber;
    
    public ClimberControlCommand(ClimberSubsystem Climber, double ClimberVoltage) {
        this.ClimberPower = ClimberVoltage;
        this.m_Climber = Climber;
    } 

    public void initialize() {
         m_Climber.ClimberControl(ClimberPower);
    }

public void execute() {
      }

public boolean isFinished() {
       return true;
       }
}
