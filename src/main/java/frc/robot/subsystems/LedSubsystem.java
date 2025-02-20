package frc.robot.subsystems;

import java.time.Period;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private boolean isFMSCONNECTED = false;
    private boolean isRobotEnabled = false;
    private boolean isRobotDisabled = false;
    private boolean isAutonomous = false;
    private boolean isTeleop = false;
    private boolean isTest = false;

    public LedSubsystem() {
        // Set the default state of the LEDs
        isFMSCONNECTED = false;
        isRobotEnabled = false;
        isRobotDisabled = true;
        isAutonomous = false;
        isTeleop = false;
        isTest = false;
    }
    public void periodic() {
        isFMSCONNECTED = DriverStation.isFMSAttached();
        isRobotEnabled = DriverStation.isEnabled();
        isRobotDisabled = DriverStation.isDisabled();
        isAutonomous = DriverStation.isAutonomous();
        isTeleop = DriverStation.isTeleop();
        isTest = DriverStation.isTest();
    }
}
