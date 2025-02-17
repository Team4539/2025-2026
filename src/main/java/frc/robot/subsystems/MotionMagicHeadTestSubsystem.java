package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotionMagicHeadTestSubsystem extends SubsystemBase {
    TalonFX head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);

    public MotionMagicHeadTestSubsystem() {
        var talonFXconfigs = new TalonFXConfiguration();

        var slot0configs = talonFXconfigs.Slot0;
        slot0configs.kS = Constants.HeadRotator.kS; // Voltage to Overcome Static Friction
        slot0configs.kV = Constants.HeadRotator.kV; // output per unit of target velocity (output/rps)
        slot0configs.kA = Constants.HeadRotator.kA; // output per unit of target acceleration (output/rps^2)
        slot0configs.kP = Constants.HeadRotator.kP; // output per unit of error in position (output/rotations)
        slot0configs.kI = Constants.HeadRotator.kI; // output per unit of integrated error in position (output/(rotations*second))
        slot0configs.kD = Constants.HeadRotator.kD; // output per unit of error in velocity (output/rps)

        var motionMagicConfigs = talonFXconfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.HeadRotator.CruiseVelocity; // velocity in units/100ms
        motionMagicConfigs.MotionMagicAcceleration = Constants.HeadRotator.Acceleration; // acceleration in units/100ms^2
        motionMagicConfigs.MotionMagicJerk = Constants.HeadRotator.Jerk; // Jerk in units/100ms^3

        // apply configs
        head.getConfigurator().apply(talonFXconfigs);
        head.setNeutralMode(NeutralModeValue.Brake);
    }

    final PositionVoltage headPositionVoltage = new PositionVoltage(0).withSlot(0);

    @Override
    public void periodic() {
        Angle position = head.getPosition().getValue();
        AngularVelocity velocity = head.getVelocity().getValue();

        SmartDashboard.putNumber("Head Position {MOTION MAGIC}", position.baseUnitMagnitude() / Constants.HeadRotator.HeadGearRatio);
        SmartDashboard.putNumber("Head Velocity {MOTION MAGIC}", velocity.baseUnitMagnitude());
    }

    public void setHeadRotation(double rotation) {
        head.setControl(headPositionVoltage.withPosition(rotation));
    }

    public void ManHead(double speed) {
        head.set(speed);
    }

    public void Stop() {
        head.set(0);
    }
}