package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateIntaketo extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_setpoint;
    private final String m_command;
    private final PIDController m_pidController;
    private final boolean m_needSelection;
    
    /**
     * Creates a command to rotate the intake to a specific angle using PID control.
     * 
     * @param intake The intake subsystem
     * @param setpoint The target position for the intake
     * @param command A descriptive name for logging
     * @param needSelection Whether the command needs special handling
     */
    public RotateIntaketo(IntakeSubsystem intake, double setpoint, String command, boolean needSelection) {
        m_intake = intake;
        m_setpoint = setpoint;
        m_command = command;
        m_needSelection = needSelection;
        
        // Configure PID controller
        m_pidController = new PIDController(3, 0.0, 0.05);
        m_pidController.setTolerance(0.02);  // Position tolerance
        
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        // Reset the PID controller when the command starts
        m_pidController.reset();
    }
    
    @Override
    public void execute() {
        // Get current position
        double currentPosition = m_intake.getRotatorPosition();
        
        // Calculate output using PID
        double output = m_pidController.calculate(currentPosition, m_setpoint);
        
        // Limit output to reasonable values
        output = Math.min(Math.max(output, -0.45), 0.45);
        
        // Apply calculated output to the intake rotator
        m_intake.rotateIntake(output, m_command);
        m_intake.runIntake(1, m_command);
        
        // Debug info
        SmartDashboard.putNumber("Intake Target Position", m_setpoint);
        SmartDashboard.putNumber("Intake PID Output", output);
        SmartDashboard.putBoolean("Intake At Setpoint", m_pidController.atSetpoint());
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the intake rotator when the command ends
        m_intake.stopRotator();
        m_intake.runIntake(0, "NO");
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when the intake reaches the setpoint
        return m_pidController.atSetpoint();
    }
}