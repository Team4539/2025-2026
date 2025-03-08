package frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private boolean m_isExtended = false; // Track if intake is extended or retracted
    private boolean m_isToggling = false;
    private final double ROTATE_SPEED = 0.5;
    private final double ROTATION_TIME = 0.7; // seconds
    private long m_startTime;
    private static final double INTAKE_SPEED = 0.7; // Speed for running the intake
    private boolean m_isFinished = false;
    
    public ToggleIntake(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        // Get current state before changing it
        m_isExtended = !m_isExtended; // Toggle the state
        m_isToggling = true;
        m_startTime = System.currentTimeMillis();
        m_isFinished = false;
    }
    
    @Override
    public void execute() {
        if (m_isToggling) {
            // Handle the rotation transition
            if (m_isExtended) {
                // Rotate intake out
                m_intakeSubsystem.rotateIntake(-ROTATE_SPEED, "extend");
            } else {
                // Rotate intake in
                m_intakeSubsystem.rotateIntake(ROTATE_SPEED, "retract");
            }
            
            // Check if we're done with the toggle animation
            if ((System.currentTimeMillis() - m_startTime) >= (ROTATION_TIME * 1000)) {
                m_isToggling = false;
                m_intakeSubsystem.rotateIntake(0, "stop"); // Stop rotation
                
                // If we're retracting (not extended), we're finished
                if (!m_isExtended) {
                    m_isFinished = true;
                }
            }
        } else {
            // After toggling is complete, handle the continuous operation
            if (m_isExtended) {
                // If extended, run the intake continuously
                m_intakeSubsystem.runIntake(INTAKE_SPEED, "intaking");
            } else {
                // If retracted, stop the intake
                m_intakeSubsystem.stopIntake();
                m_isFinished = true;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // If command is interrupted, stop everything
            m_intakeSubsystem.rotateIntake(0, "stop"); // Stop rotation
            m_intakeSubsystem.stopIntake(); // Stop intake
        } else if (!m_isExtended) {
            // If this is a normal end after retracting
            m_intakeSubsystem.rotateIntake(0, "stop"); // Stop rotation
            m_intakeSubsystem.stopIntake(); // Stop intake
        }
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}