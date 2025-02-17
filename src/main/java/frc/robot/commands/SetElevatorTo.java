package frc.robot.commands;

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
    private final boolean m_needSelection;

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
        m_needSelection = needSelection;
    }

    @Override
    public void execute() {
        double Height = m_elevator.GetElevatorHeight();
        double output = pidController.calculate(Height, m_setpoint) / 1000;
        if (output > 1) {
            fixedOutput = 1;
        } else {
            fixedOutput = output;
        }

        if (Height > m_setpoint) {
            m_elevator.SetElevator(-fixedOutput, m_command);
        } else if (Height < m_setpoint) {
            m_elevator.SetElevator(-fixedOutput, m_command);
        } else {
            m_elevator.SetElevator(0, m_command + " At Setpoint");
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.SetElevator(0, "Disabled");
    }
}