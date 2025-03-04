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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevator;
    private final MotionMagicVoltage m_motionMagicRequest;
    private double elevatorHeight;
    private String command = "disabled";

    public ElevatorSubsystem() {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevator.getConfigurator().apply(new TalonFXConfiguration());
        m_motionMagicRequest = new MotionMagicVoltage(0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure feedback settings
        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 45.0;

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
        slot0.kI = 0.0; // Integral gain
        slot0.kD = 0.5; // Derivative gain

        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevator.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportError("Failed to configure elevator motor: " + status.toString(), false);
        }
        SmartDashboard.putBoolean("Elevator Enabled", status.isOK());
    }

    @Override
    public void periodic() {
        elevatorHeight = elevator.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Height", elevatorHeight);
        SmartDashboard.putString("Elevator Command", command);
    }

    public void setPosition(double targetPosition) {
        double clampedTarget = Math.min(Math.max(targetPosition, 
                                    Constants.Elevator.ElevatorMinHeight),
                                    Constants.Elevator.ElevatorMaxHeight);
        
        elevator.setControl(m_motionMagicRequest.withPosition(clampedTarget).withSlot(0));
    }

    public void setElevator(double speed, String command, Boolean NeedDown) {
        this.command = command;
        if (speed != 0) {
           if (elevatorHeight >= Constants.Elevator.ElevatorMinHeight && elevatorHeight <= Constants.Elevator.ElevatorMaxHeight) {
               elevator.set(speed);
           } else if (elevatorHeight < Constants.Elevator.ElevatorMinHeight) {
               elevator.set(.1);
               DriverStation.reportError("Elevator To low", false);
           } else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight) {
               elevator.set(-.1);
                DriverStation.reportError("YOU ARE TOO HIGH", false);
           }
        } else {
            elevator.set(0);}
    }

    public void stopElevator() {
        elevator.set(0);
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }
}