package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimber extends Command {
    private final ClimberSubsystem m_climber;
    private final double m_speed;

    public SetClimber(ClimberSubsystem subsystem, double speed) {
        m_climber = subsystem;
        m_speed = speed;
    }

    @Override
    public void execute() {
        m_climber.runClimber(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.runClimber(0);
    }
}