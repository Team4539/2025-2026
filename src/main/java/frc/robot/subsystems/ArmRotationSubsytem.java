package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotationSubsytem extends SubsystemBase {
    TalonFX m_armMotor;
    DutyCycleEncoder m_armEncoder;
    public Boolean isSafe;
    public double armRotation;

    public ArmRotationSubsytem() {
        m_armMotor = new TalonFX(0);
        m_armEncoder = new DutyCycleEncoder(0);
        isSafe = false;
    }

    public void periodic() {
        armRotation = m_armEncoder.get() * 100;
        if (armRotation >= Constants.ArmRotator.HeadSAFE)
            isSafe = true;
        else
            isSafe = false;
    }

    public void SetArmRotation(double speed, String command) {
            m_armMotor.set(speed);
    }
    public double GetArmEncodor() {
        return armRotation;
    }
    public Boolean isSafe() {
        return isSafe;
    }
}
