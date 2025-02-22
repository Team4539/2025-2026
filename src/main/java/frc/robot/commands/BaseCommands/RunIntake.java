package frc.robot.commands.BaseCommands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    private final IntakeSubsystem  m_intakeSubsystem;
    private final double m_speed;
    private final String m_command;

    
    public RunIntake(IntakeSubsystem subsystem, double speed, String command) {
        addRequirements(subsystem);
        m_intakeSubsystem = subsystem;
        m_speed = speed;
        m_command = command;
    }
    public void execute () {
        m_intakeSubsystem.runIntake(m_speed, m_command);
    }
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }
}
