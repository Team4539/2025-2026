package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateIntake extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final double m_speed;
    private final String m_command;

    public RotateIntake(IntakeSubsystem subsystem, double speed, String command) {
        m_intakeSubsystem = subsystem;
        m_speed = speed;
        m_command = command;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.rotateIntake(m_speed, m_command);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopRotator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}