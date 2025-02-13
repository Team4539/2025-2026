package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDPSubsytem {
    PowerDistribution pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
    private double totalCurrent;

    public PDPSubsytem() {
    }

    public void periodic() {
        totalCurrent = pdp.getTotalCurrent();
        SmartDashboard.putNumber("Current", totalCurrent);
    }
}
