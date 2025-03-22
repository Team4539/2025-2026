package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class reefAlignHorizontal extends Command
{
    private CommandSwerveDrivetrain m_subsystem;
    private PhotonVision m_vision;

    private double VISION_kP = 0.01;     // TODO: Requires Testing
    private double GOAL_THRESHOLD = 0.5; // TODO: Requires Testing

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public reefAlignHorizontal(CommandSwerveDrivetrain subsystem, PhotonVision vision) 
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_vision = vision;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        double speed = 0.0;
        double targetYaw = 0.0;
        boolean visibleTarget = false;

        var result = m_vision.getBestTarget();
        if (result != null)
        {
            targetYaw = result.getYaw();
            visibleTarget = true;
        }

        if (visibleTarget)
        {
            speed = -targetYaw * VISION_kP;
        }

        m_subsystem.setControl(
            drive.withVelocityY(0.0)
                 .withVelocityX(speed)
                 .withRotationalRate(0.0)
        );
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
        var result = m_vision.getBestTarget();

        if (result != null) 
        {
            return Math.abs(result.getYaw()) < GOAL_THRESHOLD; 
        }

        return false;
    }

}
