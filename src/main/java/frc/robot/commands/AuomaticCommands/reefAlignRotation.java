package frc.robot.commands.AuomaticCommands;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class reefAlignRotation extends Command
{
    private CommandSwerveDrivetrain m_subsystem;
    private PhotonCamera m_camera;

    private double VISION_TURN_kP = 0.03;                           // TODO: Requires Testing
    private double VISION_STRAFE_kP = 0.03;                         // TODO: Requires Testing
    
    private double VISION_DES_ANGLE_deg = 0.0;                      // Goal Rotation in radians
    private double VISION_DES_RANGE_m = -1;                          // TODO: Requires Testing; Goal Distance in meters
    private double kMaxAngularSpeed = 4.0;                          // in meters
    private double kMaxLinearSpeed = 4.0;                           // in meters
    private List<Integer> allowed_apritags = Arrays.asList(6,7,8,9,10,11,17,18,19,20,21,22); 

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public reefAlignRotation(CommandSwerveDrivetrain subsystem, PhotonCamera camera) 
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_camera = camera;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        double turn = 0.0;
        double forward = 0.0;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        boolean visibleTarget = false;

        var results = m_camera.getAllUnreadResults();
        if (!results.isEmpty())
        {
            var result = results.get(results.size() - 1);
            if (result.hasTargets())
            {
                for (var target : result.getTargets())
                {
                    if (allowed_apritags.contains(target.getFiducialId()))
                    {
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                            Units.inchesToMeters(18.5),           // Camera height in meters
                            Units.inchesToMeters(8.75),         // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
                            Units.degreesToRadians(0.0),      // The pitch of the camera from the horizontal plane in radians. Positive values up.
                            Units.degreesToRadians(target.getPitch())
                            );
                            
                            visibleTarget = true;
                    }
                }
            }

            if (visibleTarget)
            {
                turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * kMaxAngularSpeed;
                forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * kMaxLinearSpeed;
            }

            m_subsystem.setControl(
                drive.withVelocityY(forward)
                .withVelocityX(0.0)
                .withRotationalRate(turn)
            );
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_subsystem.setControl(
            drive.withVelocityY(0.0)
            .withVelocityX(0.0)
            .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }

}
