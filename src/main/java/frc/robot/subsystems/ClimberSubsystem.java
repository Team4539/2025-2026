package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ClimberSubsystem {
    private TalonFX m_climberMotor;
    private TalonFXConfiguration m_climberConfig;

    public ClimberSubsystem() {
        m_climberMotor = new TalonFX(Constants.Climber.ClimberMotorID);
        m_climberConfig = new TalonFXConfiguration();
        m_climberConfig.FutureProofConfigs = true;
        m_climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
        m_climberConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        

    }

    public void runClimber(double speed) {
        m_climberMotor.set(speed);
    }


}
