package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorTo extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_setpoint;

    public SetElevatorTo(ElevatorSubsystem subsystem, double setpoint) {
        m_elevator = subsystem;
        m_setpoint = setpoint;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_elevator.getElevatorHeight() - m_setpoint) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopElevator();
    }
}