package frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmHasCoral extends Command {
    public static Command ArmupCommand (
        ElevatorSubsystem elevatorSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        ArmRotationSubsytem armRotationSubsystem) {
            return new SequentialCommandGroup(new ParallelCommandGroup(   // Return to safe position when button released
                new SetElevatorTo(elevatorSubsystem, 2.640380859375),
                new SetCarrigeTo(carrigeSubsystem, -4.3, "cause i can")).withTimeout(1),
                //new SetArmTo(armRotationSubsystem, 11.2, "Home", false)),
                new ParallelCommandGroup( new SetElevatorTo(elevatorSubsystem, 2.640380859375),
                new SetCarrigeTo(carrigeSubsystem, -4.3, "cause i can"),
                new SetArmTo(armRotationSubsystem, 16.2, "Home", false)).withTimeout(3)
            );
    }
}