package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HeadRotationSubsystem extends SubsystemBase {
    
    private DutyCycleEncoder headEncoder;
    private TalonFX head;
    private double headEncoderVal;
    private String command ="Disabled";


    public HeadRotationSubsystem() {
        headEncoder = new DutyCycleEncoder(Constants.HeadRotator.HeadRotatorEncoderID);
        head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);
        head.setNeutralMode(NeutralModeValue.Brake);
        
    }

    @Override
    public void periodic() {
        headEncoderVal = headEncoder.get()*100;
        SmartDashboard.putNumber("Head Rotation", headEncoderVal);
        SmartDashboard.putString("Head Command", command);
        
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