package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putBoolean("CoralL3", true);
      return new SequentialCommandGroup(
        new ParallelCommandGroup(   // Coral L3 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetArmTo(armRotationSubsystem, 56.2, "L3", false),
                new SetCarrigeTo(carrigeSubsystem, 1.25244140625, "cause")
            ).withTimeout(3),
            new ParallelCommandGroup(   // Coral L3 - Positioning
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetArmTo(armRotationSubsystem, 56.2, "L3", false),
                new SetCarrigeTo(carrigeSubsystem, 1.25244140625, "cause i can")
            ));
   }
   public static Command getOnFalsCommand(
        ElevatorSubsystem elevatorSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        ArmRotationSubsytem armRotationSubsystem,
        HeadintakeManipulator headManipSubsystem
   )
    {
        SmartDashboard.putBoolean("CoralL3", false);
        return new SequentialCommandGroup(
            new SetArmTo(armRotationSubsystem, 48.0, "coral ", false).withTimeout(1),
            new ParallelCommandGroup(   // Coral L3 - Output
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 1.061279296875, "cause i can"),
                new SetArmTo(armRotationSubsystem, 60, "coral L4", false),
                new RunHeadManip(headManipSubsystem, -.5)
            ).withTimeout(1),
        new ArmHasCoral().ArmupCommand(elevatorSubsystem, carrigeSubsystem, armRotationSubsystem));
    }

}
