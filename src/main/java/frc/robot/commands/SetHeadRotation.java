package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.HeadRotationSubsystem;

public class SetHeadRotation extends Command {

    private final HeadRotationSubsystem m_headRotate;
    private final DoubleSupplier m_speed;

    public SetHeadRotation(DoubleSupplier speed, HeadRotationSubsystem subsystem) 
    {
        addRequirements(subsystem);
        m_speed = speed;
        m_headRotate = subsystem;
        
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        m_headRotate.SetHeadRotation(m_speed.getAsDouble(), "Manual control");
    }
    @Override
    public void end(boolean interrupted) 
    {
        m_headRotate.SetHeadRotation(0.0, "Disabled");
    }
    
    @Override
    public boolean isFinished() { return false; }

    @Override
    public boolean runsWhenDisabled() { return false; }
}
