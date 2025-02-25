package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarrigeSubsystem extends SubsystemBase {
    private final TalonFX carrige;
    private final Encoder carrigeEncoder;
    private final MotionMagicVoltage m_motionMagicRequest;
    private double carrigeHeight;
    private String command = "disabled";
    public boolean isCarrigeUp = false;
    public boolean isArmSafe = false;

    public CarrigeSubsystem() {
        carrige = new TalonFX(Constants.Carrige.CarrigeMotorID);
        carrige.getConfigurator().apply(new TalonFXConfiguration());
        carrigeEncoder = new Encoder(Constants.Carrige.CarrigeEncoderAID, Constants.Carrige.CarrigeEncoderBID);
        m_motionMagicRequest = new MotionMagicVoltage(0);

        // Configure the motor for motion magic
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure feedback settings
        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 45.0; // Adjust if needed for your gear ratio

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

        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = carrige.getConfigurator().apply(config);
            
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportError("Failed to configure carriage motor: " + status.toString(), false);

        }
        SmartDashboard.putBoolean("Carrige initialized", status.isOK());
    }

    @Override
    public void periodic() {
        carrigeHeight = carrigeEncoder.getRaw();
        if (carrigeHeight >= Constants.Carrige.CarriageUpHeight) {
            isCarrigeUp = true;
        } else {
            isCarrigeUp = false;
        }
        SmartDashboard.putNumber("Carrige Height", carrigeHeight);
        SmartDashboard.putString("Carrige Command", command);
    }

    public void setPosition(double targetPosition) {
        // Clamp target position to valid range
        double clampedTarget = Math.min(Math.max(targetPosition, 
                                    Constants.Carrige.CarrigeMinHeight),
                                    Constants.Carrige.CarrigeMaxHeight);
                                
        carrige.setControl(m_motionMagicRequest.withPosition(clampedTarget).withSlot(0));
    }

    public void SetCarrige(double speed, String command) {
        this.command = command;
        if (speed != 0) {
            if (carrigeHeight <= Constants.Carrige.CarrigeMaxHeight && 
                carrigeHeight >= Constants.Carrige.CarrigeMinHeight) {
                carrige.set(speed);
            } else if (carrigeHeight < Constants.Carrige.CarrigeMinHeight) {
                carrige.set(-0.1);
                DriverStation.reportError("Carrige Too low", false);
            } else if (carrigeHeight > Constants.Carrige.CarrigeMaxHeight) {
                carrige.set(0.1);
                DriverStation.reportError("Carrige Too high", false);
            } else {
                carrige.set(0);
            }
        }
    }

    public void StopCarrige() {
        carrige.set(0);
    }

    public double GetCarrigeHeight() {
        return carrigeHeight;
    }

    public boolean IsCarrigeUp() {
        return isCarrigeUp;
    }
}