package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeManipulaotrSubsystem extends SubsystemBase {
    private SparkMax algaeMotor;
    private String Command;

    public AlgaeManipulaotrSubsystem() {
        algaeMotor = new SparkMax(Constants.HeadMechanisms.AlageManipulatorMotorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config 
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);
        algaeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}
    public void periodic() {
        SmartDashboard.putString("AlgaeMotor Command", "disabled");
    }
    public void SetAlgaeManipulator(double speed, String command) {
        SmartDashboard.putString("AlgaeMotor Command", command);
        if (speed != 0) {
            algaeMotor.set(speed);
        }
        else {
            algaeMotor.set(0);
        }
    }
}
