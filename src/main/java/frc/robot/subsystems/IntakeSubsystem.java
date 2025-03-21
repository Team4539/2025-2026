package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeRotatorMotor;
    private final Ultrasonic intakeUltrasonic;
    private double intakeRotatorPosition;
    private String intakeRotatorCommand;
    private String intakeCommand;
    private boolean isIntakeMoving;
    private double IntakeDistance;
    private double intakerotation;
    private DutyCycleEncoder m_intakeEncoder;
    private TalonFXConfiguration m_intakeconfig;

    @SuppressWarnings("static-access")
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
        m_intakeconfig = new TalonFXConfiguration();
        m_intakeconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(m_intakeconfig);
        intakeRotatorMotor = new TalonFX(Constants.Intake.IntakeRotatorMotorID);
        intakeRotatorMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeRotatorCommand = "Disabled";
        intakeCommand = "Disabled";
        isIntakeMoving = false;
        intakeUltrasonic = new Ultrasonic(9, 8); 
        intakeUltrasonic.setAutomaticMode(true);
        m_intakeEncoder = new DutyCycleEncoder(1);
    }

    public void periodic() {
        IntakeDistance = intakeUltrasonic.getRangeInches();
        intakerotation = m_intakeEncoder.get();
        SmartDashboard.putNumber("Intake Rotator Position", intakeRotatorPosition);
        SmartDashboard.putString("Intake Rotator Command", intakeRotatorCommand);
        SmartDashboard.putString("Intake Command", intakeCommand);
        SmartDashboard.putNumber("intakeDistance", IntakeDistance);
        SmartDashboard.putNumber("Intake Encoder", m_intakeEncoder.get());
        
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
        double currentAngle = m_intakeEncoder.get();
        if (currentAngle >= Constants.Intake.IntakeRotatorMinAngle && currentAngle <= Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(speed);}
         else if (currentAngle < Constants.Intake.IntakeRotatorMinAngle & speed > 0)
            intakeRotatorMotor.set(speed);
         else if (currentAngle > Constants.Intake.IntakeRotatorMinAngle) {
            intakeRotatorMotor.set(-0.1);
            DriverStation.reportError("i am too low ", false);
        } else if (currentAngle < Constants.Intake.IntakeRotatorMaxAngle & speed < 0) {
            intakeRotatorMotor.set(speed);
        }
        else if (currentAngle < Constants.Intake.IntakeRotatorMaxAngle) {
            intakeRotatorMotor.set(0.1);
            DriverStation.reportError("i am too high", false);
        // } else {
            stopRotator();
        }
    }

    public void stopRotator() {
        intakeRotatorMotor.set(0);
    }

    public double getRotatorPosition() {
        return m_intakeEncoder.get();
    }
    public boolean isIntakeMoving() {
        return isIntakeMoving;
    }
}