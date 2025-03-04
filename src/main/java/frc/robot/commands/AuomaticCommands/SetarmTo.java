package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotationSubsytem;

public class SetArmTo extends Command {
    private final ArmRotationSubsytem m_arm;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;
    private TrapezoidProfile.State currentState;
    private String m_command;
    private double m_setpoint;
    private double fixedOutput;
    private double startTime;

    public SetArmTo(ArmRotationSubsytem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_arm = subsystem;
        m_command = (command == null) ? "Disabled" : command;
        m_setpoint = setpoint;
        
        // Adjusted PID values for smoother control
        pidController = new PIDController(0.055, 0, 0.01);
        
        // Further reduced acceleration for smoother transitions
        constraints = new TrapezoidProfile.Constraints(
            150.0,  // Max velocity
            80.0   // Max acceleration
        );
    }

    @Override
    public void initialize() {
        // Create new profile when command starts
        startState = new TrapezoidProfile.State(m_arm.GetArmRotation(), 0.0);
        endState = new TrapezoidProfile.State(m_setpoint, 0.0);
        profile = new TrapezoidProfile(constraints);
        startTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
public void execute() {
    double currentTime = System.currentTimeMillis() / 1000.0;
    double elapsedTime = currentTime - startTime;
    
    currentState = profile.calculate(elapsedTime, startState, endState);
    
    double angle = m_arm.GetArmRotation();
    double output = pidController.calculate(angle, currentState.position);
    
    // Adjusted FF value based on observed behavior
    double gravityFF = 0.095 * Math.cos(Math.toRadians(angle));
    double velocityFF = 0.01 * currentState.velocity;
    double accelFF = 0.0025 *profile.calculate(elapsedTime +.02, startState, endState).velocity;
    
    // In execute()
    if (Math.abs(currentState.position - endState.position) < 10.0) {
        // Near target, start reducing velocity requirement
        double slowdownFactor = Math.abs(currentState.position - endState.position) / 10.0;
        velocityFF *= slowdownFactor;
    }
    
    output += gravityFF + velocityFF + accelFF;
    
    // Direction-specific output limits
    double maxOutputUp = 1.0;   // When moving up
    double maxOutputDown = 0.8; // When moving down

    if (output > 0) {
        fixedOutput = Math.min(output, maxOutputUp);
    } else {
        fixedOutput = Math.max(output, -maxOutputDown);
    }
    
    m_arm.SetArm(-fixedOutput, m_command);
    
    // Debug info
    SmartDashboard.putNumber("Profile Position", currentState.position);
    SmartDashboard.putNumber("Profile Velocity", currentState.velocity);
    SmartDashboard.putNumber("Current Angle", angle);
    SmartDashboard.putNumber("Output", fixedOutput);
    SmartDashboard.putNumber("Feed Forward", gravityFF);
    SmartDashboard.putNumber("Velocity FF", velocityFF);

}

    public void periodic() {
        double angle = m_arm.GetArmRotation();
        double gravityFF = 0.12 * Math.cos(Math.toRadians(angle));
        SmartDashboard.putNumber("Profile Position", currentState.position);
        SmartDashboard.putNumber("Profile Velocity", currentState.velocity);
        SmartDashboard.putNumber("Current Angle", angle);
        SmartDashboard.putNumber("Output", fixedOutput);
        SmartDashboard.putNumber("Feed Forward", gravityFF);

    }

    @Override
    public void end(boolean interrupted) {
        m_arm.SetArm(0, "Disabled");
    }

    @Override
    public boolean isFinished() {
    double elapsedTime = System.currentTimeMillis() / 1000.0 - startTime;
    return profile.isFinished(elapsedTime) 
        && Math.abs(m_arm.GetArmRotation() - m_setpoint) < .5  // Increased tolerance
        && Math.abs(currentState.velocity) < 0.5;  // Add velocity check
}
}