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
    private boolean isEstopped = false;

    public LedSubsystem() {
        // Set the default state of the LEDs
        isFMSCONNECTED = false;
        isRobotEnabled = false;
        isRobotDisabled = true;
        isAutonomous = false;
        isTeleop = false;
        isTest = false;
        isEstopped = false;
    }
    public void periodic() {
        isFMSCONNECTED = DriverStation.isFMSAttached();
        isRobotEnabled = DriverStation.isEnabled();
        isRobotDisabled = DriverStation.isDisabled();
        isAutonomous = DriverStation.isAutonomous();
        isTeleop = DriverStation.isTeleop();
        isTest = DriverStation.isTest();
        isEstopped = DriverStation.isEStopped();
    }
    public void setLEDs() {
        
        if (isRobotEnabled) {
            if (isAutonomous) {
                // Set LEDs to Blue
            } else if (isTeleop) {
                // Set LEDs to Green
            } else if (isTest) {
                // Set LEDs to Yellow
            }
        } else if (isRobotDisabled) {
            // Set LEDs to Red
        } else if (isEstopped) {
            // Set LEDs to Flashing Red
        } else {
            // Set LEDs to White
        }
    }
}
