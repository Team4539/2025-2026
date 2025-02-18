package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightHelpers;

public class AlignReef extends Command {
    private final com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric forwardStraight = new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
        
    private final CommandSwerveDrivetrain m_drive;
    private int m_tag;

    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double tx = LimeLightHelpers.getTX("limelight-fronapr");
        double ty = LimeLightHelpers.getTY("limelight-fronapr");
        double ta = LimeLightHelpers.getTA("limelight-fronapr");
        double id = LimeLightHelpers.getFiducialID("limelight-fronapr");

        if (id == m_tag) {
            double kP = .015; 
            double targetingAngularVelocity = tx * kP;
            targetingAngularVelocity *= 1.5 * Math.PI;
            targetingAngularVelocity *= -1.0;

            double kPStrafe = 0.1;
            double strafeVelocity = ty * kPStrafe;

            double kPForward = 0.1;
            double forwardVelocity = ta * kPForward;

            m_drive.setControl(forwardStraight
                .withRotationalRate(targetingAngularVelocity)
                .withVelocityY(strafeVelocity)
                .withVelocityX(forwardVelocity));
        } else {
            m_drive.setControl(forwardStraight
                .withRotationalRate(0.0)
                .withVelocityY(0.0)
                .withVelocityX(0.0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setControl(forwardStraight
            .withRotationalRate(0)
            .withVelocityY(0)
            .withVelocityX(0));
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public boolean runsWhenDisabled() { return false; }
}