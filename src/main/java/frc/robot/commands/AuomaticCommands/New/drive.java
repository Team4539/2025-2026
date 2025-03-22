package frc.robot.commands.AuomaticCommands.New;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class drive extends Command
{
    private CommandSwerveDrivetrain m_subsystem;
    private double m_x_speed;
    private double m_y_speed;
    private double m_rotation;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public drive(double x_speed, double y_speed, double rotation, CommandSwerveDrivetrain subsystem) 
    {
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_x_speed = x_speed;
        m_y_speed = y_speed;
        m_rotation = rotation;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        m_subsystem.setControl(
            drive.withVelocityY(m_y_speed)
            .withVelocityX(m_x_speed)
            .withRotationalRate(m_rotation)
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
        return false;
    }

}
