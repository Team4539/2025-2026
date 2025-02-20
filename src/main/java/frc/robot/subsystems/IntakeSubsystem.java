package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeRotatorMotor;
    private final DutyCycleEncoder intakeRotatorEncoder;
    private double intakeRotatorPosition;
    private String intakeRotatorCommand;
    private String intakeCommand;


    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
        intakeRotatorMotor = new TalonFX(Constants.Intake.IntakeRotatorMotorID);
        intakeRotatorEncoder = new DutyCycleEncoder(Constants.Intake.IntakeRotatorEncoderID);
        intakeRotatorCommand = "Disabled";
        intakeCommand = "Disabled";
    }

    public void periodic() {
        intakeRotatorPosition = intakeRotatorEncoder.get();
        SmartDashboard.putNumber("Intake Rotator Position", intakeRotatorPosition);
        SmartDashboard.putString("Intake Rotator Command", intakeRotatorCommand);
        SmartDashboard.putString("Intake Command", intakeCommand);
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
        if (currentAngle >= Constants.Intake.IntakeRotatorMinAngle && currentAngle <= Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(speed);}
        else if (currentAngle < Constants.Intake.IntakeRotatorMinAngle) {
            intakeRotatorMotor.set(-0.1);
        } else if (currentAngle > Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(0.1);
        } else {
            stopRotator();
        }
    }

    public void stopRotator() {
        intakeRotatorMotor.set(0);
    }

    public double getRotatorPosition() {
        return intakeRotatorEncoder.get();
    }

}