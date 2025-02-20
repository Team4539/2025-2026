package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotationSubsytem;

public class SetArmTo extends Command {
    private final ArmRotationSubsytem m_arm;
    private final PIDController pidController;
    private String m_command;
    private double m_setpoint;
    private double fixedOutput;

    public SetArmTo(ArmRotationSubsytem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_arm = subsystem;
        if (command == null) {
            m_command = "Disabled";
        } else {
            m_command = command;
        }
        m_setpoint = setpoint;
        pidController = new PIDController(.6, 10, 0.02);
    }

    @Override
    public void execute() {
        double angle = m_arm.GetArmRotation();
        double output = pidController.calculate(angle, m_setpoint) / 1000;
        if (output > 1) {
            fixedOutput = 1;
        } else {
            fixedOutput = output;
        }

        if (angle > m_setpoint) {
            m_arm.SetArm(-fixedOutput, m_command);
        } else if (angle < m_setpoint) {
            m_arm.SetArm(-fixedOutput, m_command);
        } else {
            m_arm.SetArm(0, m_command + " At Setpoint");
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.SetArm(0, "Disabled");
    }
}