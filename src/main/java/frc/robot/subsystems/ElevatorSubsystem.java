package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private  TalonFX elevator;
    private Encoder elevatorEncoder;
    private double elevatorHeight;
    private String comand = "disabled";
    private boolean isSAFE;
    private HeadRotationSubsystem m_HeadRotationSubsystem;

    @SuppressWarnings("removal")
    public ElevatorSubsystem(HeadRotationSubsystem headRotationSubsystem) {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevatorEncoder = new Encoder(Constants.Elevator.ElevatorEncoderAID, Constants.Elevator.ElevatorEncoderBID);
        elevator.getConfigurator().apply(new TalonFXConfiguration());
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 20;
        currentLimits.SupplyCurrentLimitEnable = true;
        elevator.getConfigurator().apply(currentLimits);
        elevator.setInverted(true);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        m_HeadRotationSubsystem = headRotationSubsystem;
        isSAFE = m_HeadRotationSubsystem.isSafe();


    }

    public void SetStatus(String command) {
        comand = command;
    }

    @Override
    public void periodic() {
        elevatorHeight = elevatorEncoder.getRaw();
        SmartDashboard.putNumber("Elevator Height", elevatorEncoder.getRaw());
        SmartDashboard.putString("Elevator Command", comand);
        isSAFE = m_HeadRotationSubsystem.isSafe();

    }

    public void SetElevator(double speed, String comand) {
        SmartDashboard.putString("Elevator Command", comand);
        if (speed != 0) {
            if (isSAFE || isSAFE == false && elevatorHeight <= Constants.Elevator.ElevatorSAFE) {
                if ((elevatorHeight >= Constants.Elevator.ElevatorMaxHeight && elevatorHeight <= Constants.Elevator.ElevatorMinHeight)) {
                    elevator.set(speed);
                }
                else if (elevatorHeight < Constants.Elevator.ElevatorMinHeight){
                    elevator.set(-.1);
                    DriverStation.reportError("YOU ARE TO LOW", false);
                }
                else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight){
                    elevator.set(.1);
                    DriverStation.reportError("YOU ARE TO HIGH", false);
                }
            }
            else {
                elevator.set(0);
                DriverStation.reportError("YOU ARE NOT SAFE", false);
            }
        } 
        else {
            elevator.set(0);
        }
    }
    public double GetElevatorHeight() {
        return elevatorHeight;
    }
}
