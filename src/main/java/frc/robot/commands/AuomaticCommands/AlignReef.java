package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignReef extends Command {
    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private CommandSwerveDrivetrain m_drive;
    private int m_tag;
    private PhotonCamera camera;
    
    // PID Controllers for alignment
    private final PIDController rotationPID;
    private final PIDController strafePID;
    private final PIDController distancePID;

    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;
        camera = new PhotonCamera("fronapr"); // Use your actual camera name here

        // Configure PID controllers with gains
        rotationPID = new PIDController(0.015, 0, 0);
        strafePID = new PIDController(0.015, 0, 0);
        distancePID = new PIDController(0.1, 0, 0);

        // Set tolerances for when we consider ourselves "aligned"
        rotationPID.setTolerance(1.0);
        strafePID.setTolerance(1.0);
        distancePID.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        strafePID.reset();
        distancePID.reset();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            
            // Check if we see the correct AprilTag
            if (target.getFiducialId() == m_tag) {
                // Get yaw (horizontal offset) and pitch (vertical offset)
                double tx = target.getYaw();
                double ty = target.getPitch();

                // Calculate control outputs
                double rotationOutput = rotationPID.calculate(tx, 0) * 1.5 * Math.PI;
                double strafeOutput = strafePID.calculate(tx, 0);
                double forwardOutput = distancePID.calculate(ty, 0);

                // Apply combined movement
                m_drive.setControl(drive
                    .withVelocityX(forwardOutput)  // Forward/backward
                    .withVelocityY(strafeOutput)   // Left/right
                    .withRotationalRate(rotationOutput*-1)); // Rotation
            } else {
                stopMovement();
            }
        } else {
            stopMovement();
        }
    }

    private void stopMovement() {
        m_drive.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we're aligned with the target
        return rotationPID.atSetpoint() && 
               strafePID.atSetpoint() && 
               distancePID.atSetpoint();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}