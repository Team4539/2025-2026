package frc.robot.commands.AuomaticCommands.ScoringPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class Barge {
    public static Command getOnTrueCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem) {
        
        return 
            new ParallelCommandGroup(   // Coral L4 - Positioning
                new SetArmTo(armRotationSubsystem, 63.1, "coral L4", false),
                new SetCarrigeTo(carrigeSubsystem,-4.3 , null),
                new SetElevatorTo(elevatorSubsystem, 4.8)
            );
}}
