package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDPSubsytem extends SubsystemBase{
    PowerDistribution pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
    private double totalCurrent;
    private double Voltage;

    public PDPSubsytem() {
    }

    public void periodic() {
        totalCurrent = pdp.getTotalCurrent();
        Voltage = pdp.getVoltage();
        SmartDashboard.putNumber("Voltage", Voltage);
        SmartDashboard.putNumber("Current", totalCurrent);

    }
}
