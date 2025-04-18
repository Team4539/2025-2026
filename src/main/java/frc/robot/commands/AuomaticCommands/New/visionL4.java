package frc.robot.commands.AuomaticCommands.New;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AuomaticCommands.reefAlignHorizontal;
import frc.robot.commands.AuomaticCommands.reefAlignRotation;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL4;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;

public class visionL4 
{
    public static Command run(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CarrigeSubsystem carriage, ArmRotationSubsytem armRotation, HeadintakeManipulator head, PhotonVision vision, PhotonCamera camera, IntakeSubsystem intake)
    {
        return Commands.sequence(
            Commands.parallel(
                CoralL4.getOnTrueCommand(elevator, carriage, armRotation),
                Commands.sequence(
                    new reefAlignRotation(drivetrain, camera).withTimeout(2),
                    new reefAlignHorizontal(drivetrain, vision)
                )
            ).withTimeout(10),
            Commands.parallel(
                CoralL4.getOnFalseCommand(elevator, carriage, armRotation, head, drivetrain)
                )
        );
    }
}