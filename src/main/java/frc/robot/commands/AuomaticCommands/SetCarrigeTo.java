package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarrigeSubsystem;

public class SetCarrigeTo extends Command {
    private final CarrigeSubsystem m_carrige;
    private final double m_setpoint;
    private final String m_command;

    public SetCarrigeTo(CarrigeSubsystem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_carrige = subsystem;
        m_setpoint = setpoint;
        m_command = command != null ? command : "Disabled";
    }

    @Override
    public void initialize() {
        m_carrige.setPosition(m_setpoint);
    }

    @Override
    public void execute() {
        // Motion magic handles the movement automatically
    }

    @Override
    public boolean isFinished() {
        // Add tolerance for position check
        double currentPosition = m_carrige.GetCarrigeHeight();
        return Math.abs(currentPosition - m_setpoint) < 0.1; // Adjust tolerance as needed
    }

    @Override
    public void end(boolean interrupted) {
        m_carrige.StopCarrige();
    }
}