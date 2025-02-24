package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotationSubsytem extends SubsystemBase {
    private final TalonFX m_armMotor;
    private final DutyCycleEncoder m_armEncoder;
    private final MotionMagicVoltage m_motionMagicRequest;
    private final PIDController m_pidController;
    public Boolean isoutside;
    public double armRotation;
    private boolean isCarrigeUp;
    private int printCount = 0;
    // Add encoder offset
    private double encoderOffset;

    public ArmRotationSubsytem() {
        m_armMotor = new TalonFX(16);
        m_armEncoder = new DutyCycleEncoder(2);
        m_motionMagicRequest = new MotionMagicVoltage(0);
        isoutside = false;
        m_pidController = new PIDController(0.2, 0, 0.2);
        
        // Store initial encoder position as offset
        encoderOffset = m_armEncoder.get();

        // Configure the motor for motion magic
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure feedback ratio
        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = Constants.ArmRotator.RotationPerDegree;

        
        // Configure Motion Magic parameters
        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(50))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(1000));
        
        // Configure PID and feedforward gains
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.25; // Static friction compensation
        slot0.kV = 0.12; // Velocity feedforward
        slot0.kA = 0.01; // Acceleration feedforward
        slot0.kP = 60.0; // Position proportion gain
        slot0.kI = 0.0;  // Integral gain
        slot0.kD = 0.5;  // Derivative gain
        
        // Apply configuration with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_armMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportError("Failed to configure arm motor: " + status.toString(), false);
        }
    }

    @Override
    public void periodic() {
        // Calculate armRotation relative to initial position
        armRotation = (m_armEncoder.get()) - encoderOffset;
        
        // Update dashboard values
        if (++printCount >= 10) {
            printCount = 0;
            SmartDashboard.putNumber("armRotation", armRotation);
            SmartDashboard.putNumber("encoderOffset", encoderOffset);
            SmartDashboard.putNumber("rawEncoder", m_armEncoder.get());
        }
        
        // Check position against offset-adjusted HeadOutside value
        if (armRotation >= Constants.ArmRotator.HeadOutside) {
            isoutside = true;
        } else {
            isoutside = false;
        }
    }

    public void setPosition(double targetPosition) {
        // Clamp target position to valid range, considering offset
        double offsetAdjustedTarget = targetPosition + encoderOffset;
        offsetAdjustedTarget = Math.min(Math.max(offsetAdjustedTarget, 
                                     Constants.ArmRotator.ArmRotatorMinAngle + encoderOffset),
                                     Constants.ArmRotator.ArmRotatorMaxAngle + encoderOffset);
                                
        m_armMotor.setControl(m_motionMagicRequest.withPosition(offsetAdjustedTarget).withSlot(0));
    }

    public void ManArm(double speed) {
        m_armMotor.set(speed);
    }

    public void HomeArm() {
   
    }

    public double GetArmRotation() {
        // Return position relative to initial position
        return m_armMotor.getPosition().getValueAsDouble() + encoderOffset;
    }

    public Boolean isSafe() {
        // Check safety against offset-adjusted values
        return armRotation <= Constants.ArmRotator.HeadSAFE;
    }
}