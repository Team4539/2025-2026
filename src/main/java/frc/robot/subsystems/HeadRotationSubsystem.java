package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HeadRotationSubsystem extends SubsystemBase {
    
    private Spark headEncoder;
    private TalonFX head;



    public HeadRotationSubsystem() {
        headEncoder = new Spark(Constants.HeadRotator.HeadRotatorEncoderID);
        head = new TalonFX(Constants.HeadRotator.HeadRotatorMotorID);
        head.setNeutralMode(NeutralModeValue.Brake);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Head Rotation", headEncoder.get());
        
    }

    public void SetHeadRotation(double speed) {
        
    }
    public double GetHeadEncodor() {
        return headEncoder.get();
    }
}