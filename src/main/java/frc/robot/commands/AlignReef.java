package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightHelpers;
import edu.wpi.first.math.geometry.Pose3d;

public class AlignReef extends Command {
    private final com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric forwardStraight = new com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
        
    private final CommandSwerveDrivetrain m_drive;
    private int m_tag;
    private boolean isAligned = false;

    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;
    }

    @Override
    public void initialize() {
        isAligned = false;
    }

    @Override
    public void execute() {
        double tx = LimeLightHelpers.getTX("limelight-fronapr");
        double ty = LimeLightHelpers.getTY("limelight-fronapr");
        double id = LimeLightHelpers.getFiducialID("limelight-fronapr");

        if (id == m_tag) {
            Pose3d botPose = LimeLightHelpers.getBotPose3d("limelight-fronapr");
            double yaw = botPose.getRotation().getZ(); // Get the current yaw of the robot

            double kPStrafe = 0.05; // Sensitivity for strafe adjustment
            double strafeVelocity = tx * kPStrafe;

            double kPYaw = 0.05; // Sensitivity for yaw adjustment
            double yawAdjustment = -yaw * kPYaw;

            // Deadband to prevent small corrections
            if (Math.abs(tx) < 1.0) {
                strafeVelocity = 0.0;
            }

            // Adjust yaw to be parallel to the April tag
            if (Math.abs(yaw) < Math.toRadians(1.0)) {
                yawAdjustment = 0.0;
                isAligned = true;
            }

            m_drive.setControl(forwardStraight
                .withRotationalRate(yawAdjustment)
                .withVelocityY(strafeVelocity) // Adjusted strafe velocity
                .withVelocityX(0.0)); // No forward movement until aligned
        } else {
            m_drive.setControl(forwardStraight
                .withRotationalRate(0.0)
                .withVelocityY(0.0)
                .withVelocityX(0.0)); // Stop if tag is not detected
        }

        // If aligned, drive forward
        if (isAligned) {
            double kPForward = 0.1; // Sensitivity for forward movement
            double forwardVelocity = ty * kPForward;

            m_drive.setControl(forwardStraight
                .withRotationalRate(0.0)
                .withVelocityY(0.0)
                .withVelocityX(forwardVelocity)); // Drive forward
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