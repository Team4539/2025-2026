package frc.robot.subsystems;

import org.opencv.video.SparsePyrLKOpticalFlow;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeRotatorMotor;
    private final Encoder intakeRotatorEncoder;
    private double intakeRotatorPosition;
    private String intakeRotatorCommand;
    private String intakeCommand;
    private boolean isIntakeMoving;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
        intakeRotatorMotor = new TalonFX(Constants.Intake.IntakeRotatorMotorID);
        intakeRotatorEncoder = new Encoder(Constants.Intake.IntakeRotatorEncoderID, 6);
        intakeRotatorCommand = "Disabled";
        intakeCommand = "Disabled";
        isIntakeMoving = false;
    }

    public void periodic() {
        intakeRotatorPosition = intakeRotatorEncoder.get();
        SmartDashboard.putNumber("Intake Rotator Position", intakeRotatorPosition);
        SmartDashboard.putString("Intake Rotator Command", intakeRotatorCommand);
        SmartDashboard.putString("Intake Command", intakeCommand);
        if (intakeCommand != "Disabled" || intakeRotatorCommand != "Disabled") {
            isIntakeMoving = true;
        } else {
            isIntakeMoving = false;
        }
        
    }

    public void runIntake(double speed, String command) {
        intakeCommand = command;
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
        intakeCommand = "STOPPED";
    }

    public void rotateIntake(double speed, String command) {
        intakeRotatorCommand = command;
        double currentAngle = intakeRotatorEncoder.get();
        if (currentAngle <= Constants.Intake.IntakeRotatorMinAngle && currentAngle >= Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(speed);}
         else if (currentAngle > Constants.Intake.IntakeRotatorMinAngle & speed < 0)
            intakeRotatorMotor.set(speed);
         else if (currentAngle > Constants.Intake.IntakeRotatorMinAngle) {
            intakeRotatorMotor.set(-0.1);
        } else if (currentAngle < Constants.Intake.IntakeRotatorMaxAngle & speed > 0) {
            intakeRotatorMotor.set(speed);
        }
        else if (currentAngle < Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(0.1);
        // } else {
            stopRotator();
        }
    }

    public void stopRotator() {
        intakeRotatorMotor.set(0);
    }

    public double getRotatorPosition() {
        return intakeRotatorEncoder.get();
    }
    public boolean isIntakeMoving() {
        return isIntakeMoving;
    }
}