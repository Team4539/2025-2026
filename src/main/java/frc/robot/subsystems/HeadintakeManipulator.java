package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadintakeManipulator extends SubsystemBase {
    private TalonFX m_headIntakeMotor;
    private double m_speed;

    public HeadintakeManipulator() {
        m_headIntakeMotor = new TalonFX(17);
        m_speed = 0;
    }

    public void runHeadIntake(double speed) {
        m_speed = speed;
        m_headIntakeMotor.set(speed);
    }
}
