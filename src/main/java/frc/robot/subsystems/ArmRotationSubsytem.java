package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotationSubsytem extends SubsystemBase {
    TalonFX m_armMotor;
    DutyCycleEncoder m_armEncoder;
    public Boolean isoutside;
    public double armRotation;
    private boolean isCarrigeUp;


    public ArmRotationSubsytem() {
        CarrigeSubsystem carrigeSubsystem = new CarrigeSubsystem();
        m_armMotor = new TalonFX(0);
        m_armEncoder = new DutyCycleEncoder(0);
        isoutside = false;
        isCarrigeUp = carrigeSubsystem.IsCarrigeUp();
    }

    public void periodic() {
        armRotation = m_armEncoder.get() * 100;
        if (armRotation >= Constants.ArmRotator.HeadOutside)
            isoutside = true;
        else
            isoutside = false;
    }

    public void SetArm(double speed, String command) {
        if (speed != 0) {
            if (isCarrigeUp) {
                if (isoutside) {
                    m_armMotor.set(speed);
                } else {
                    m_armMotor.set(-.1);
                }
            } else {
                m_armMotor.set(0);
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
}
