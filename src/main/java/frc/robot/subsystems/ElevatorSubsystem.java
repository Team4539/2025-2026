package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX elevator;
    private DutyCycleEncoder elevatorEncoder;

    public ElevatorSubsystem() {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevatorEncoder = new DutyCycleEncoder(Constants.Elevator.ElevatorEncoderID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", elevatorEncoder.get());
    }

    public void SetElevator(double speed) {
        if (speed != 0) {
            if ((elevatorEncoder.get() < Constants.Elevator.ElevatorMaxHeight) && (elevatorEncoder.get() > Constants.Elevator.ElevatorMinHeight)) {
                elevator.set(speed);
            }
            else if (elevatorEncoder.get() < Constants.Elevator.ElevatorMinHeight){
                elevator.set(-1);
            }
            else if (elevatorEncoder.get() > Constants.Elevator.ElevatorMaxHeight){
                elevator.set(1);
            }
        } 
        else {
            elevator.set(0);
        }
    }
    public double GetElevatorHeight() {
        return elevatorEncoder.get();
    }
}
