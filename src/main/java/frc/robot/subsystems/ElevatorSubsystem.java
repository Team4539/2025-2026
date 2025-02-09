package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private  TalonFX elevator;
    private Encoder elevatorEncoder;
    private double elevatorHeight;

    public ElevatorSubsystem() {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevatorEncoder = new Encoder(Constants.Elevator.ElevatorEncoderAID, Constants.Elevator.ElevatorEncoderBID);
        elevator.setNeutralMode(NeutralModeValue.Brake);

    }



    @Override
    public void periodic() {
        elevatorHeight = elevatorEncoder.getRaw();
        SmartDashboard.putNumber("Elevator Height", elevatorEncoder.getRaw());
        SmartDashboard.putString("Command", "Disabled");
    }

    public void SetElevator(double speed, String comand) {
        SmartDashboard.putString("Elevator Command", comand);
        if (speed != 0) {
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
        }
    }
    public double GetElevatorHeight() {
        return elevatorHeight;
    }
}
