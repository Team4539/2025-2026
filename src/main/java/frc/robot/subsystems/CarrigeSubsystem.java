package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarrigeSubsystem extends SubsystemBase {
    private TalonFX carrige;
    private Encoder carrigeEncoder;
    private double carrigeHeight;
    private String command = "disabled";

    public CarrigeSubsystem() {
        carrige = new TalonFX(Constants.Carrige.CarrigeMotorID);
        carrigeEncoder = new Encoder(Constants.Carrige.CarrigeEncoderAID, Constants.Carrige.CarrigeEncoderBID);
        carrige.getConfigurator().apply(new TalonFXConfiguration());
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 20;
        currentLimits.SupplyCurrentLimitEnable = true;
        carrige.getConfigurator().apply(currentLimits);
        carrige.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        carrigeHeight = carrigeEncoder.getRaw();
        SmartDashboard.putNumber("Carrige Height", carrigeEncoder.getRaw());
        SmartDashboard.putString("Carrige Command", command);
    }

    public void SetCarrige(double speed, String command) {
        SmartDashboard.putString("Carrige Command", command);

        if (speed != 0) {
            if ((carrigeHeight >= Constants.Carrige.CarrigeMaxHeight && carrigeHeight <= Constants.Carrige.CarrigeMinHeight)) {
                carrige.set(speed);
            } else if (carrigeHeight < Constants.Carrige.CarrigeMinHeight) {
                carrige.set(-.1);
                DriverStation.reportError("YOU ARE TOO LOW", false);
            } else if (carrigeHeight > Constants.Carrige.CarrigeMaxHeight) {
                carrige.set(.1);
                DriverStation.reportError("YOU ARE TOO HIGH", false);
            }
        } else {
            carrige.set(0);
        }
    }

    public double GetCarrigeHeight() {
        return carrigeHeight;
    }
}