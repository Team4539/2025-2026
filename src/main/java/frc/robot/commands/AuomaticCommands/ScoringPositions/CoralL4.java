package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmNeedsCoral;
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
        SmartDashboard.putBoolean("CoralL4", true);
        
        return new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L4 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 4.403564453125),
                new SetCarrigeTo(carrigeSubsystem, 0.844970703125, "cause")
            ).withTimeout(5),
            new ParallelCommandGroup(   // Coral L4 - Positioning
                new SetArmTo(armRotationSubsystem, 57.3, "coral L4", false),
                new SetCarrigeTo(carrigeSubsystem, 0.844970703125, null),
                new SetElevatorTo(elevatorSubsystem, 4.403564453125)
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
            new ParallelCommandGroup(   // Coral L4 - Positioning and score than run back to a safe position
                new SetArmTo(armRotationSubsystem, 43.9, "coral L4", false),
                new SetElevatorTo(elevatorSubsystem, 4.478271484375)
            ).withTimeout(.7),
        new SetCarrigeTo(carrigeSubsystem, 1.14599609375, null).withTimeout(.5),
        new RunHeadManip(headManipSubsystem, -.2).withTimeout(1),
        new ParallelRaceGroup(
            new ArmHasCoral().ArmupCommand(elevatorSubsystem, carrigeSubsystem, armRotationSubsystem), 
            new RunHeadManip(headManipSubsystem, -.5))
        );
            
    }
}
