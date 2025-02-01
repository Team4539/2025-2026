package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HeadRotationSubsystem extends SubsystemBase {
    
    private SparkMax headEncoder;
    private TalonFX head;



    public HeadRotationSubsystem() {
        headEncoder = new SparkMax(Constants.HeadRotator.HeadRotatorEncoderID, MotorType.kBrushed);
        head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);
        head.setNeutralMode(NeutralModeValue.Brake);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Head Rotation", headEncoder.getAbsoluteEncoder().getPosition());
        
    }

    public void SetHeadRotation(double speed) {
        
    }
    public double GetHeadEncodor() {
        return headEncoder.getAbsoluteEncoder().getPosition();
    }
}