package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotationSubsytem;

public class SetArmTo extends Command {
    private final ArmRotationSubsytem m_arm;
    private final double m_setpoint;
    private final String m_command;

    public SetArmTo(ArmRotationSubsytem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_arm = subsystem;
        m_setpoint = setpoint;
        m_command = (command != null) ? command : "Disabled";
    }

    @Override
    public void execute() {
        // Use the built-in Motion Magic control
        m_arm.setPosition(m_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        // Motion Magic will maintain position when command ends
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted
        return false;
    }
}