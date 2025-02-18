package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotionMagicElevatorTestSubsystem extends SubsystemBase {
    TalonFX elevator = new TalonFX(Constants.Elevator.ElevatorMotorID);
    Encoder encoder = new Encoder(5, 6);

    public MotionMagicElevatorTestSubsystem() {
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXconfigs.Feedback.FeedbackRemoteSensorID = encoder.get();
        

        var slot0configs = talonFXconfigs.Slot0;
        slot0configs.kS = Constants.Elevator.kS; // Voltage to Overcome Static Friction
        slot0configs.kV = Constants.Elevator.kV; // output per unit of target velocity (outout/rps)
        slot0configs.kA = Constants.Elevator.kA; // output per unit of target acceleration (output/rps^2)
        slot0configs.kP = Constants.Elevator.kP; // output per unit of error in position (output/rotations)
        slot0configs.kI = Constants.Elevator.kI; // output per unit of integraed error in position (output/(rotations*second))
        slot0configs.kD = Constants.Elevator.kD; // output per unit of error in velocity (output/rps)
        
        var motionMagicConfigs = talonFXconfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.CruiseVelocity; //velocity in units/100ms
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.Acceleration; // acceleration in units/100ms^2
        motionMagicConfigs.MotionMagicJerk = Constants.Elevator.Jerk; // Jerk in units/100ms^3
    

        // apply configs
        elevator.getConfigurator().apply(talonFXconfigs);
        elevator.setNeutralMode(NeutralModeValue.Brake);
    }
    final PositionVoltage elevatorPositionVoltage = new PositionVoltage(0).withSlot(0);

    @Override
    public void periodic() {
        
        Angle position = elevator.getPosition().getValue();
        AngularVelocity velocity = elevator.getVelocity().getValue();

        SmartDashboard.putNumber("Elevator Position {MOTION MAGIC}", position.baseUnitMagnitude()/ Constants.Elevator.ElevatorGearRatio);
        SmartDashboard.putNumber("Elevator Velocity {MOTION MAGIC}", velocity.baseUnitMagnitude());
    }


    public void setElevatorHeight(double height) {
        elevator.setControl(elevatorPositionVoltage.withPosition(height));
    }

    public void ManElevator(double speed) {
        elevator.set(speed);
    }


    public void Stop() {
        elevator.set(0);
    }
}