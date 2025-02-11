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

public class HeadRotationSubsystem extends SubsystemBase {
    
    private DutyCycleEncoder headEncoder;
    private TalonFX head;
    private double headEncoderVal;


    public HeadRotationSubsystem() {
        headEncoder = new DutyCycleEncoder(Constants.HeadRotator.HeadRotatorEncoderID);
        head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);
        head.getConfigurator().apply(new TalonFXConfiguration());
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 20;
        currentLimits.SupplyCurrentLimitEnable = true;
        head.getConfigurator().apply(currentLimits);
        head.setNeutralMode(NeutralModeValue.Brake);
        headEncoder.isConnected();
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Head Rotation", headEncoderVal);
        SmartDashboard.putString("Head Command", "disabled");
        headEncoderVal = headEncoder.get();
        
    }

    public void SetHeadRotation(double speed, String command) {
       SmartDashboard.putString("Head Command", command);
        if (speed != 0) {
            if ((headEncoderVal >= Constants.HeadRotator.HeadRotatorMaxAngle && headEncoder.get() <= Constants.HeadRotator.HeadRotatorMinAngle)) {
                head.set(speed);
            }
            else if (headEncoderVal < Constants.HeadRotator.HeadRotatorMinAngle){
                head.set(-.1);
            }
            else if (headEncoderVal > Constants.HeadRotator.HeadRotatorMaxAngle){
                head.set(.1);
            }
            else 
                head.set(0);
            }
            
    }
    public double GetHeadEncodor() {
        return headEncoder.get();
    }
}