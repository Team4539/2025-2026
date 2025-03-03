package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;

public class CoralL3 extends Command{
    public static Command getOnTruecCommand(
        ElevatorSubsystem elevatorSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        ArmRotationSubsytem armRotationSubsystem
    ) {
      return new SequentialCommandGroup(
        new ParallelCommandGroup(   // Coral L3 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 2.337646484375, "cause")
            ).withTimeout(3),
            new ParallelCommandGroup(   // Coral L3 - Positioning
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 2.337646484375, "cause i can")
            ));
   }
   public static Command getOnFalsCommand(
        ElevatorSubsystem elevatorSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        ArmRotationSubsytem armRotationSubsystem,
        HeadintakeManipulator headManipSubsystem
   )
    {
        return new SequentialCommandGroup(
            new SetArmTo(armRotationSubsystem, 48.0, "coral L4", false).withTimeout(1),
            new ParallelCommandGroup(   // Coral L3 - Output
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 3.04833984375, "cause i can"),
                new SetArmTo(armRotationSubsystem, 60, "coral L4", false),
                new RunHeadManip(headManipSubsystem, .5)
            ).withTimeout(.5),
        new ArmHasCoral().ArmupCommand(elevatorSubsystem, carrigeSubsystem, headManipSubsystem, armRotationSubsystem));
    }

}
