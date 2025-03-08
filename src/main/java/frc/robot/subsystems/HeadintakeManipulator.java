package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadintakeManipulator extends SubsystemBase {
    private TalonFX m_headIntakeMotor;
    private TalonFXConfiguration m_headIntakeConfig;
    private double m_speed;

    public HeadintakeManipulator() {
        m_headIntakeMotor = new TalonFX(Constants.HeadMechanisms.HeadManipulatorMotorID);
        m_headIntakeConfig = new TalonFXConfiguration();
        m_headIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_headIntakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        
        // Apply configuration with retry mechanism
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_headIntakeMotor.getConfigurator().apply(m_headIntakeConfig);
            if (status.isOK()) break;
        }
        
        // Report error if configuration failed
        if (!status.isOK()) {
            DriverStation.reportError("Failed to configure head intake motor: " + status.toString(), false);
        }
        
        // Display status on SmartDashboard
        SmartDashboard.putBoolean("HeadIntake Initialized", status.isOK());
        m_speed = 0;
    }

    public void runHeadIntake(double speed) {
        m_speed = speed;
        m_headIntakeMotor.set(speed);
    }
    
    @Override
    public void periodic() {
        // Optionally add telemetry here
        SmartDashboard.putNumber("HeadIntake Speed", m_speed);
    }
    
    // Add a stop method for consistency with other subsystems
    public void stopHeadIntake() {
        m_speed = 0;
        m_headIntakeMotor.set(0);
    }
}