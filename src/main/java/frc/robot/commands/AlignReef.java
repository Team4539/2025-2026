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
        double id = LimeLightHelpers.getFiducialID("limelight-fronapr");

        if (id == m_tag) {
            Pose3d botPose = LimeLightHelpers.getBotPose3d("limelight-fronapr");
            double tx = botPose.getX(); // Get the X position of the robot
            double ty = botPose.getY(); // Get the Y position of the robot
            double tz = botPose.getZ(); // Get the Z position of the robot
            double yaw = botPose.getRotation().getZ(); // Get the current yaw of the robot

            double kPStrafe = 0.05; // Sensitivity for strafe adjustment
            double strafeVelocity = tx * kPStrafe;

            double kPForward = 0.1; // Sensitivity for forward movement
            double forwardVelocity = tz * kPForward;

            double kPYaw = 0.05; // Sensitivity for yaw adjustment
            double yawAdjustment = -yaw * kPYaw;

            // Deadband to prevent small corrections
            if (Math.abs(tx) < 0.1) {
                strafeVelocity = 0.0;
            }
            if (Math.abs(tz) < 0.1) {
                forwardVelocity = 0.0;
            }

            // Adjust yaw to be parallel to the April tag
            if (Math.abs(yaw) < Math.toRadians(1.0)) {
                yawAdjustment = 0.0;
            }

            // Check if the robot is aligned
            if (Math.abs(tx) < 0.1 && Math.abs(tz) < 0.1 && Math.abs(yaw) < Math.toRadians(1.0)) {
                isAligned = true;
            } else {
                isAligned = false;
            }

            m_drive.setControl(forwardStraight
                .withRotationalRate(yawAdjustment)
                .withVelocityY(strafeVelocity * -1) // Adjusted strafe velocity
                .withVelocityX(forwardVelocity)); // Adjusted forward velocity
        } else {
            m_drive.setControl(forwardStraight
                .withRotationalRate(0.0)
                .withVelocityY(0.0)
                .withVelocityX(0.0)); // Stop if tag is not detected
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
    public boolean isFinished() { return isAligned; }

    @Override
    public boolean runsWhenDisabled() { return false; }
}