package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarrigeSubsystem;

public class SetCarrige extends Command {
    private final CarrigeSubsystem m_carrige;
    private final double m_speed;
    private final String m_command;

    public SetCarrige(CarrigeSubsystem subsystem, double speed, String command) {
        addRequirements(subsystem);
        m_carrige = subsystem;
        m_speed = speed;
        m_command = command;
    }

    @Override
    public void execute() {
        m_carrige.SetCarrige(m_speed, m_command);
    }

    @Override
    public void end(boolean interrupted) {
        m_carrige.StopCarrige();
    }
}
