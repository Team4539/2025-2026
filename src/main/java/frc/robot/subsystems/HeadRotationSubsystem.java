package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;

public class HeadRotationSubsystem extends SubsystemBase {
    
    private  TalonFX head;
    private DutyCycleEncoder headEncoder;
    private double HeadRotation;

    public HeadRotationSubsystem() {
        head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);
        headEncoder = new DutyCycleEncoder(Constants.HeadRotator.HeadRotatorEncoderID); 
        head.getConfigurator().apply(new TalonFXConfiguration());
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40;
        currentLimits.SupplyCurrentLimitEnable = true;
        head.getConfigurator().apply(currentLimits);
        head.setNeutralMode(NeutralModeValue.Brake);
        Elastic.Notification notification = new Elastic.Notification();
    }



    @Override
    public void periodic() {
        HeadRotation = headEncoder.get()*100;
        SmartDashboard.putNumber("Head Rotation", HeadRotation);
        SmartDashboard.putString("Head Command", "disabled");
        
    }

    public void SetHeadRotation(double speed, String comand) {
        SmartDashboard.putData("Run Head Motor", head);
        SmartDashboard.putString("Head Command", comand);
        if (speed != 0) {
            if ((HeadRotation >= Constants.HeadRotator.HeadRotatorMaxAngleOnGround && HeadRotation <= Constants.HeadRotator.HeadRotatorMinAngleOnGround)) {
                head.set(speed);
            }
            else if (HeadRotation < Constants.HeadRotator.HeadRotatorMinAngleOnGround){
                head.set(.1);
                DriverStation.reportError("Trying to rotate to far back", false);
            }
            else if (HeadRotation > Constants.HeadRotator.HeadRotatorMaxAngleOnGround){
                head.set(-.1);
                DriverStation.reportError("Tryign to rotate to far forward", false);
            }
        } 
        else {
            head.set(0);
        }
    }
    public double GetHeadEncodor() {
        return HeadRotation;
    }
}
