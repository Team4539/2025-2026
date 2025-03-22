package frc.robot.commands.AuomaticCommands.New;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AuomaticCommands.reefAlignHorizontal;
import frc.robot.commands.AuomaticCommands.reefAlignRotation;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL4;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVision;

public class visionL4 
{
    public static Command run(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CarrigeSubsystem carriage, ArmRotationSubsytem armRotation, PhotonVision vision, PhotonCamera camera)
    {
        return Commands.parallel(
            CoralL4.getOnTrueCommand(elevator, carriage, armRotation),
            Commands.sequence(
                new WaitCommand(1),
                new reefAlignRotation(drivetrain, camera).withTimeout(5),
                new drive(0.0, 0.3, 0.0, drivetrain).withTimeout(1),
                new reefAlignHorizontal(drivetrain, vision)
            )
        );
    }
}