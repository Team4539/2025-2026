package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.New.drive;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
        SmartDashboard.putBoolean("CoralL2", true);
        
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmTo(armRotationSubsystem, 60.4, "hold", false),
                new SequentialCommandGroup(
                    Commands.waitSeconds(.5),
                new ParallelCommandGroup(   // Coral L2 - Positioning
                new SetElevatorTo(elevatorSubsystem, 0),
                new SetCarrigeTo(carrigeSubsystem, -0.3, "cause i can"))
            )
            
            )
        );
    }
    
    /**
     * Gets the command to run when the button is released (returning to safe position)
     */
    public static Command getOnFalseCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem,
            HeadintakeManipulator headManipSubsystem, 
            CommandSwerveDrivetrain drive) {
                
        SmartDashboard.putBoolean("CoralL2", false);
        return new SequentialCommandGroup(
            new SetArmTo(armRotationSubsystem, 53.2, "coral L2", false).withTimeout(1),
            new ParallelCommandGroup(new RunHeadManip(headManipSubsystem, -.4).withTimeout(5),
            new drive(0, .5, 0, drive)).withTimeout(1));
            
    }
}