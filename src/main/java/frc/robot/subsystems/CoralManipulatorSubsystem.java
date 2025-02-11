package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulatorSubsystem  extends SubsystemBase {
    private SparkMax coralMotor;
    private boolean isForCoral;

    public CoralManipulatorSubsystem() {
        coralMotor = new SparkMax(Constants.HeadMechanisms.CoralManipulatorMotorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config 
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        isForCoral = false;
    }   

    public void SetCoralManipulator(double speed, String command) {
        if (speed != 0) {
            coralMotor.set(speed);
        }
        else {
            coralMotor.set(0);
        }
    }
}