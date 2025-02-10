package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadRotationSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class SetHeadTo extends Command {
    private final HeadRotationSubsystem m_head;
    private final PIDController pidController;
    private final double m_setpoint;
    private final String m_command;
    private double m_fixedOutput;

    public SetHeadTo(HeadRotationSubsystem subsystem, double setpoint, String command) {
        addRequirements(subsystem);
        m_head = subsystem;
        if (command == null) {
            m_command = "Disabled";
        } else {
            m_command = command;
        }
        m_setpoint = setpoint;
       pidController = new PIDController(.1, 10, 0.02);
    }


    @Override
    public void execute(){
        double Rotation = m_head.GetHeadEncodor();
        double output = pidController.calculate(Rotation, m_setpoint)/1000;
        if (output > 1) {
            m_fixedOutput = 1;
        }
        else m_fixedOutput = output;

        if (Rotation > m_setpoint) {
            m_head.SetHeadRotation(-m_fixedOutput, m_command);
        }
        else if (Rotation <  m_setpoint) {
            m_head.SetHeadRotation(m_fixedOutput, m_command);
        }
        else{
            m_head.SetHeadRotation(0, m_command+" At Setpoint");
        }
    }
    public void end(boolean interrupted) {
        m_head.SetHeadRotation(0, "Disabled");
    }
}  
