package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX elevator;
    private Encoder elevatorEncoder;
    private double elevatorHeight;
    private String command = "disabled";

    @SuppressWarnings("removal")
    public ElevatorSubsystem() {
        elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
        elevatorEncoder = new Encoder(Constants.Elevator.ElevatorEncoderAID, Constants.Elevator.ElevatorEncoderBID);
        elevator.getConfigurator().apply(new TalonFXConfiguration());
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40;
        currentLimits.SupplyCurrentLimitEnable = true;
        elevator.getConfigurator().apply(currentLimits);
        elevator.setInverted(true);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        
    }

    @Override
    public void periodic() {
        elevatorHeight = elevatorEncoder.getRaw();
        SmartDashboard.putNumber("Elevator Height", elevatorEncoder.getRaw());
        SmartDashboard.putString("Elevator Command", command);
    }

    public void setElevator(double speed, String command, Boolean NeedDown) {

        SmartDashboard.putString("Elevator Command", command);

        if (speed != 0) {
            if (NeedDown = true) {
                if ((elevatorHeight >= Constants.Elevator.ElevatorMaxHeight && elevatorHeight <= Constants.Elevator.ElevatorMinHeight)) {
                    elevator.set(speed); // I am in the range
                }
                else if (elevatorHeight < Constants.Elevator.ElevatorMinHeight && speed >= 0){
                    elevator.set(speed); // I am below but I want to go up
                    System.out.println("I'll allow you to get me up");
                }
                } else if (elevatorHeight < Constants.Elevator.ElevatorMinHeight && speed >= 0) {
                    elevator.set(-.1); // I am below and wanna go down (I am too low)
                    DriverStation.reportError("YOU ARE TOO LOW", false);}
                else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight && speed <= 0) {
                    elevator.set(speed); // I am above but i wanna go down
                    System.out.println("I'll allow you to get me down");
                } else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight) {
                    elevator.set(.1); // I am above and wanna go up (I am too high)
                    DriverStation.reportError("YOU ARE TOO HIGH", false);
                }
            else {
                if (elevatorHeight >= Constants.Elevator.ElevatorMaxHeight && elevatorHeight <= Constants.Elevator.ElevatorSAFE) {
                    elevator.set(speed);} // I am in the range
                else if (elevatorHeight < Constants.Elevator.ElevatorSAFE && speed >= 0) {
                    elevator.set(speed); // I am below but I want to go up
                    System.out.println("I'll allow you to get me up");
                } else if (elevatorHeight < Constants.Elevator.ElevatorSAFE && speed >= 0) {
                    elevator.set(-.1); // I am below and wanna go down (I am too low)
                    DriverStation.reportError("YOU ARE TOO LOW", false);}
                else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight && speed <= 0) {
                    elevator.set(speed); // I am above but i wanna go down
                    System.out.println("I'll allow you to get me down");
                } else if (elevatorHeight > Constants.Elevator.ElevatorMaxHeight && speed <= 0) {
                    elevator.set(.1);// I am above and wanna go up (I am too high)
                    DriverStation.reportError("YOU ARE TOO HIGH", false);
                }
            }
        } else {
            elevator.set(0);
        }
    }
    

    public double getElevatorHeight() {
        return elevatorHeight;
    }
}