package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorTo extends Command {
    private final ElevatorSubsystem m_elevator;
    private final PIDController pidController;
    private String m_command;
    private double m_setpoint;
    private double fixedOutput;

    public SetElevatorTo(ElevatorSubsystem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_elevator = subsystem;
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
        double Height = m_elevator.getElevatorHeight();
        double output = pidController.calculate(Height, m_setpoint) / 1000;
        if (output > 1) {
            fixedOutput = 1;
        } else {
            fixedOutput = output;
        }

        if (Height > m_setpoint) {
            m_elevator.setElevator(-fixedOutput, m_command, false);
        } else if (Height < m_setpoint) {
            m_elevator.setElevator(-fixedOutput, m_command, false);
        } else {
            m_elevator.setElevator(0, m_command + " At Setpoint", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setElevator(0, "Disabled", false);
    }
}