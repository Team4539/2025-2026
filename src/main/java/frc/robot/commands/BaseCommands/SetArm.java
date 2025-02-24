package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotationSubsytem;

public class SetArm extends Command {
    private final ArmRotationSubsytem m_arm;
    private final double m_speed;
    private final String m_command;

    public SetArm(ArmRotationSubsytem subsystem, double speed, String command) {
        addRequirements(subsystem);
        m_arm = subsystem;
        m_speed = speed;
        m_command = command;
    }

    @Override
    public void execute() {
        m_arm.ManArm(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.ManArm(0);
    }
}
