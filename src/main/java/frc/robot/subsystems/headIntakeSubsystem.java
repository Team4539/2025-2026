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

public class headIntakeSubsystem extends SubsystemBase {
    private SparkMax algaeMotor;
    private double AlgaeCurrentDraw;
    private SparkMax coralMotor;
    private String Command;

    @SuppressWarnings("deprecation")
    public headIntakeSubsystem() {
        algaeMotor = new SparkMax(Constants.HeadMechanisms.AlageManipulatorMotorID, MotorType.kBrushless);
        coralMotor = new SparkMax(Constants.HeadMechanisms.CoralManipulatorMotorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config 
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        algaeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralMotor.setInverted(true);

}
    public void periodic() {
        AlgaeCurrentDraw = algaeMotor.getOutputCurrent();
        SmartDashboard.putString("AlgaeMotor Command", "disabled");
        SmartDashboard.putNumber("Algae Current Draw", AlgaeCurrentDraw);

    }
    public void SetAlgae(double speed, String command) {
        SmartDashboard.putString("AlgaeMotor Command", command);
        if (speed != 0) {
            if (AlgaeCurrentDraw > 5) {
                algaeMotor.set(speed);
                coralMotor.set(speed);}
            else{ 
                algaeMotor.set(Constants.HeadMechanisms.AlgaeHoldSpeed);
                coralMotor.set(Constants.HeadMechanisms.AlgaeHoldSpeed);
            }
            
        }
        else {
            algaeMotor.set(0);
            coralMotor.set(0);
            
        }
    }
    public void SetCoral(double speed, String command) {
        SmartDashboard.putString("CoralMotor Command", command);
        if (speed != 0) {
            coralMotor.set(speed);
        }
        else {
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
