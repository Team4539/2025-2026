package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CarrigeSubsystem;

public class SetCarrigeTo extends Command {
    private final CarrigeSubsystem m_carrige;
    private final PIDController pidController;
    private String m_command;
    private double m_setpoint;
    private double fixedOutput;

    public SetCarrigeTo(CarrigeSubsystem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_carrige = subsystem;
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
        double Height = m_carrige.GetCarrigeHeight();
        double output = pidController.calculate(Height, m_setpoint) / 1000;
        if (output > 1) {
            fixedOutput = 1;
        } else {
            fixedOutput = output;
        }

        if (Height > m_setpoint) {
            m_carrige.SetCarrige(-fixedOutput, m_command);
        } else if (Height < m_setpoint) {
            m_carrige.SetCarrige(-fixedOutput, m_command);
        } else {
            m_carrige.SetCarrige(0, m_command + " At Setpoint");
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_carrige.SetCarrige(0, "Disabled");
    }
}