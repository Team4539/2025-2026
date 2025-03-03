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

public class CoralL2 extends Command {
    
    /**
     * Gets the command to run when the button is pressed (moving to coral L2 position)
     */
    public static Command getOnTrueCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem) {
        
        return new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L2 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 4.295654296875, "cause"),
                new SetArmTo(armRotationSubsystem, 60, "hold", false)
            ).withTimeout(3),
            new ParallelCommandGroup(   // Coral L2 - Positioning
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, 4.295654296875, "cause i can")
            ).withTimeout(1)
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
            new SetArmTo(armRotationSubsystem, 45.3, "coral L2", false).withTimeout(1),
            new ParallelCommandGroup(   // Return to intake position
                new SetElevatorTo(elevatorSubsystem, 2.919677734375),
                new SetCarrigeTo(carrigeSubsystem, 0, "cause i can"),
                new SetArmTo(armRotationSubsystem, 14, "Home", false),
                new RunHeadManip(headManipSubsystem, .5)
            ).withTimeout(3)
        );
    }
}