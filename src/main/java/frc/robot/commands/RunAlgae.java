package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.headIntakeSubsystem;

public class RunAlgae  extends Command {
    private final headIntakeSubsystem m_HeadIntakeSubsystem;
    private final double m_speed;
    private String m_Command = "disabled";
    
        public RunAlgae(Double Speed, String command, headIntakeSubsystem headIntakeSubsystem) {
            addRequirements(headIntakeSubsystem);
            m_HeadIntakeSubsystem = headIntakeSubsystem;
            m_speed = Speed;
            m_Command = command;
    }
    public void execute() {
        if (m_HeadIntakeSubsystem.GetAlgaeCurrentDraw() < 10) {
            m_HeadIntakeSubsystem.SetAlgae(m_speed, m_Command);
            m_HeadIntakeSubsystem.SetCoral(m_speed, m_Command);}
            else {
                m_HeadIntakeSubsystem.SetAlgae(Constants.HeadMechanisms.AlgaeHoldSpeed, "Algae Hold");
                m_HeadIntakeSubsystem.SetCoral(Constants.HeadMechanisms.AlgaeHoldSpeed, "Algae Hold");
            }
        }
    public void end (boolean interrupted) {
        m_HeadIntakeSubsystem.SetAlgae(0.0, "Disabled");
        m_HeadIntakeSubsystem.SetCoral(0.0, "Disabled");
    }
}
