package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeRotatorMotor;
    private final Encoder intakeRotatorEncoder;
    private final Ultrasonic intakeUltrasonic;
    private double intakeRotatorPosition;
    private String intakeRotatorCommand;
    private String intakeCommand;
    private boolean isIntakeMoving;
    private double IntakeDistance;

    @SuppressWarnings("static-access")
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeRotatorMotor = new TalonFX(Constants.Intake.IntakeRotatorMotorID);
        intakeRotatorMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeRotatorEncoder = new Encoder(Constants.Intake.IntakeRotatorEncoderID, 6);
        intakeRotatorCommand = "Disabled";
        intakeCommand = "Disabled";
        isIntakeMoving = false;
        intakeUltrasonic = new Ultrasonic(9, 8); 
        intakeUltrasonic.setAutomaticMode(true);
    }

    public void periodic() {
        intakeRotatorPosition = intakeRotatorEncoder.get();
        IntakeDistance = intakeUltrasonic.getRangeInches();
        SmartDashboard.putNumber("Intake Rotator Position", intakeRotatorPosition);
        SmartDashboard.putString("Intake Rotator Command", intakeRotatorCommand);
        SmartDashboard.putString("Intake Command", intakeCommand);
        SmartDashboard.putNumber("intakeDistance", IntakeDistance);
        
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