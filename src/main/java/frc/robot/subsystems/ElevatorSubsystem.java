package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX elevator;
    private SparkMax elevatorEncoder;
    private double elevatorHeight;

    public ElevatorSubsystem() {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevatorEncoder = new SparkMax(Constants.Elevator.ElevatorEncoderID, MotorType.kBrushless);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        elevatorHeight = elevatorEncoder.getEncoder().getPosition();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", elevatorHeight);
    }

    public void SetElevator(double speed) {
        if (speed != 0) {
            if ((elevatorHeight < Constants.Elevator.ElevatorMaxHeight) && (elevatorHeight > Constants.Elevator.ElevatorMinHeight)) {
                elevator.set(speed);
            }
            else if (elevatorHeight < Constants.Elevator.ElevatorMinHeight){
                elevator.set(-1);
            }
            else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight){
                elevator.set(1);
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
