package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotationSubsytem extends SubsystemBase {
    TalonFX m_armMotor;
    DutyCycleEncoder m_armEncoder;
    public Boolean isoutside;
    public double armRotation;
    private TalonFXConfiguration m_armConfig;


    public ArmRotationSubsytem() {
        m_armMotor = new TalonFX(16);
        m_armEncoder = new DutyCycleEncoder(2);
        isoutside = false;
        m_armConfig = new TalonFXConfiguration();
        m_armConfig.FutureProofConfigs = true;
        m_armConfig.CurrentLimits.SupplyCurrentLimit = 40;
        m_armConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        m_armConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_armMotor.getConfigurator().apply(m_armConfig);
            if (status == StatusCode.OK) {
                break;
            }
        }
        if (status != StatusCode.OK) {
            DriverStation.reportError("Failed to configure arm motor", false);
        }
        SmartDashboard.putBoolean("Arm initialized", status.isOK());
    }

    public void periodic() {
        armRotation = m_armEncoder.get() * 100;
        if (armRotation >= Constants.ArmRotator.HeadOutside)
            isoutside = true;
        else
            isoutside = false;
        SmartDashboard.putNumber("ArmRotation", armRotation);
    }

    public void SetArm(double speed, String command) {
        if (speed != 0) {
           if (armRotation >= Constants.ArmRotator.ArmRotatorMinAngle && armRotation <= Constants.ArmRotator.ArmRotatorMaxAngle) {
               m_armMotor.set(speed);
           } else if (armRotation < Constants.ArmRotator.ArmRotatorMinAngle) {
               m_armMotor.set(-.1);
               DriverStation.reportError("Arm To low", false);
           } else if (armRotation > Constants.ArmRotator.ArmRotatorMaxAngle) {
               m_armMotor.set(.1);
                DriverStation.reportError("YOU ARE TOO HIGH", false);
           }
        } else {
            m_armMotor.set(0);
        }
        
    }
    public double GetArmRotation() {
        return armRotation;
    }
    public Boolean isSafe() {
        return isoutside;
    }
    public void STOPARM() {
        m_armMotor.set(0);
    }
}