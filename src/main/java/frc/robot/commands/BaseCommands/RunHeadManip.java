package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadintakeManipulator;

public class RunHeadManip extends Command {
    private final HeadintakeManipulator m_headIntake;
    private final double m_speed;

    public RunHeadManip(HeadintakeManipulator subsystem, double speed) {
        addRequirements(subsystem);
        m_headIntake = subsystem;
        m_speed = speed;
    }

    @Override
    public void execute() {
        m_headIntake.runHeadIntake(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_headIntake.runHeadIntake(0);
    }
}
