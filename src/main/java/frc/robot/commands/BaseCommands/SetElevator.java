package frc.robot.commands.BaseCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command{ 
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final Double m_speed;

    public SetElevator(Double speed, ElevatorSubsystem subsystem) 
    {
        addRequirements(subsystem);
        m_speed = speed;
        m_ElevatorSubsystem = subsystem;
        
    }

    @Override
    public void execute() 
    {
        m_ElevatorSubsystem.setElevator(m_speed * -1.0, "Manual control", false);
    }
    @Override
    public void end(boolean interrupted) 
    {
        m_ElevatorSubsystem.setElevator(0.0, "Disabled", false);
    }
}