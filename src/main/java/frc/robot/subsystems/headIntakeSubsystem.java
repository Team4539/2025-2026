package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class headIntakeSubsystem extends SubsystemBase {
    private TalonFX algaeMotor;
    private Double AlgaeCurrentDraw;
    private TalonFX coralMotor;
    private String Command;
    private ElevatorSubsystem m_ElevatorSubsystem;
    private HeadRotationSubsystem m_HeadRotationSubsystem;

    public headIntakeSubsystem(ElevatorSubsystem elevatorSubsystem, HeadRotationSubsystem headRotationSubsystem) {
        m_ElevatorSubsystem = elevatorSubsystem;
        m_HeadRotationSubsystem = headRotationSubsystem;
        algaeMotor = new TalonFX(Constants.HeadMechanisms.AlageManipulatorMotorID);
        coralMotor = new TalonFX(Constants.HeadMechanisms.CoralManipulatorMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;
        algaeMotor.getConfigurator().apply(config);
        coralMotor.getConfigurator().apply(config);
        coralMotor.getConfigurator().apply(config);
        coralMotor.setInverted(true);
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        coralMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void periodic() {
        AlgaeCurrentDraw = algaeMotor.getStatorCurrent().getValueAsDouble();
        SmartDashboard.putString("AlgaeMotor Command", "disabled");
        SmartDashboard.putNumber("Algae Current Draw", AlgaeCurrentDraw);
    }

    public void SetAlgae(double speed, String command) {
        SmartDashboard.putString("AlgaeMotor Command", command);
        if (speed != 0) {
            algaeMotor.set(speed);
            coralMotor.set(speed);
        } else {
            algaeMotor.set(0);
            coralMotor.set(0);
        }
    }

    public void SetCoral(double speed, String command) {
        if (m_ElevatorSubsystem.GetElevatorHeight() > Constants.Elevator.ElevatorAboveGround || m_HeadRotationSubsystem.GetHeadEncodor() <= Constants.HeadRotator.HeadPastSAFE) {
            SmartDashboard.putString("CoralMotor Command", command);
            if (speed != 0) {
                coralMotor.set(speed);
            } else {
                coralMotor.set(0);
            }
        } else {
            DriverStation.reportError("Coral motor not moving, elevator is too low", true);
            coralMotor.set(0);
        }
    }

    public void SetStatus(String command) {
        Command = command;
    }

    public double GetAlgaeCurrentDraw() {
        return AlgaeCurrentDraw;
    }
}