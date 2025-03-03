package frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;

public class ArmNeedsCoral extends Command {
    public Command GoingtoCoral(
        ElevatorSubsystem elevatorSubsystem,
        ArmRotationSubsytem armRotationSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        HeadintakeManipulator headManipSubsystem
    )
    {
        // brings the arm down to be ready to pick up coral
        return new ParallelCommandGroup(   // Return to safe position
                new SetElevatorTo(elevatorSubsystem, 2.919677734375),
                new SetCarrigeTo(carrigeSubsystem, 0, "cause i can"),
                new SetArmTo(armRotationSubsystem, 14, "Home", false)
            ).withTimeout(5);
    }
}
