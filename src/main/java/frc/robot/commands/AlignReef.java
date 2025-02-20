package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightHelpers;

public class AlignReef extends Command {
    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private CommandSwerveDrivetrain m_drive;
    private int m_tag;
    
    // PID Controllers for alignment
    private final PIDController rotationPID;
    private final PIDController strafePID;
    private final PIDController distancePID;

    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;

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
        double tx = LimeLightHelpers.getTX("limelight-fronapr");
        double ty = LimeLightHelpers.getTY("limelight-fronapr");
        double id = LimeLightHelpers.getFiducialID("limelight-fronapr");

        if (id == m_tag) {
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
            // If we don't see the correct tag, stop moving
            m_drive.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
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