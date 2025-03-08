package frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private boolean m_isExtended = false; // Track if intake is extended or retracted
    private boolean m_isFinished = false;
    private final double ROTATE_SPEED = 0.5;
    private final double ROTATION_TIME = 0.7; // seconds
    private long m_startTime;
    
    public ToggleIntake(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        m_isExtended = !m_isExtended; // Toggle the state
        m_startTime = System.currentTimeMillis();
        m_isFinished = false;
    }
    
    @Override
    public void execute() {
        if (m_isExtended) {
            // Rotate intake out
            m_intakeSubsystem.rotateIntake(-ROTATE_SPEED, "extend");
        } else {
            // Rotate intake in
            m_intakeSubsystem.rotateIntake(ROTATE_SPEED, "retract");
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.rotateIntake(0, "stop"); // Stop rotation
    }
    
    @Override
    public boolean isFinished() {
        // End after rotation time has elapsed
        return (System.currentTimeMillis() - m_startTime) >= (ROTATION_TIME * 1000);
    }
}