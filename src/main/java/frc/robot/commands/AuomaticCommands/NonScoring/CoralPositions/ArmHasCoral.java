package frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmHasCoral extends Command {
    public Command ArmupCommand (
        ElevatorSubsystem elevatorSubsystem,
        CarrigeSubsystem carrigeSubsystem,
        ArmRotationSubsytem armRotationSubsystem) {
            return new ParallelCommandGroup(   // Return to safe position when button released
                new SetElevatorTo(elevatorSubsystem, 1.087158203125),
                new SetCarrigeTo(carrigeSubsystem, 0, "cause i can"),
                new SetArmTo(armRotationSubsystem, 60, "Home", false)
            ).withTimeout(2);
    }
}