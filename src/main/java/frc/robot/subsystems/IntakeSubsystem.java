package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeRotatorMotor;
    private final DutyCycleEncoder intakeRotatorEncoder; 

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
        intakeRotatorMotor = new TalonFX(Constants.Intake.IntakeRotatorMotorID);
        intakeRotatorEncoder = new DutyCycleEncoder(Constants.Intake.IntakeRotatorEncoderID);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void rotateIntake(double speed) {
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