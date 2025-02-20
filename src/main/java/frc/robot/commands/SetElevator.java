package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command{ 
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final DoubleSupplier m_speed;

    public SetElevator(DoubleSupplier speed, ElevatorSubsystem subsystem) 
    {
        addRequirements(subsystem);
        m_speed = speed;
        m_ElevatorSubsystem = subsystem;
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        m_ElevatorSubsystem.setElevator(m_speed.getAsDouble()*-1, "Manual control");
    }
    @Override
    public void end(boolean interrupted) 
    {
        m_ElevatorSubsystem.setElevator(0.0, "Disabled");
    }
    
    @Override
    public boolean isFinished() { return false; }

    @Override
    public boolean runsWhenDisabled() { return false; }
}
