package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;

public class CoralL4 extends Command {
    
    /**
     * Gets the command to run when the button is pressed (moving to coral L4 position)
     */
    public static Command getOnTrueCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem) {
        
        return new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L4 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 2.92919921875),
                new SetCarrigeTo(carrigeSubsystem, 0, "cause")
            ).withTimeout(5),
            new ParallelCommandGroup(   // Coral L4 - Positioning
                new SetArmTo(armRotationSubsystem, 55.8, "coral L4", false),
                new SetCarrigeTo(carrigeSubsystem, 0, null),
                new SetElevatorTo(elevatorSubsystem, 2.92919921875)
            ).withTimeout(5)
            
        );
    }
    
    /**
     * Gets the command to run when the button is released (returning to safe position)
     */
    public static Command getOnFalseCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem,
            HeadintakeManipulator headManipSubsystem) {
                
        return new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L4 - Positioning
                new SetArmTo(armRotationSubsystem, 47.1, "coral L4", false),
                new SetCarrigeTo(carrigeSubsystem, 0, null),
                new SetElevatorTo(elevatorSubsystem, 2.92919921875)
            ).withTimeout(5),
            new ParallelCommandGroup(   // Return to safe position when button released
                new SetElevatorTo(elevatorSubsystem, 1.087158203125),
                new SetCarrigeTo(carrigeSubsystem, 0, "cause i can"),
                new SetArmTo(armRotationSubsystem, 60, "Home", false),
                new RunHeadManip(headManipSubsystem, .5)
            ).withTimeout(1));
    }

    public class getOnFalseCommand {
    }
}