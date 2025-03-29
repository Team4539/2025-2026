package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class CoralL1 extends Command {

    public static Command getOnTrueCommand(
            ElevatorSubsystem elevatorSubsystem,
            
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem) {
        SmartDashboard.putBoolean("CoralL2", true);
        
        return new ParallelCommandGroup(
            new SetArmTo(armRotationSubsystem, 30, "hold", false),
         new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    Commands.waitSeconds(.5),
                new ParallelCommandGroup(   // Coral L2 - Positioning
                new SetElevatorTo(elevatorSubsystem, 2.548095703125),
                new SetCarrigeTo(carrigeSubsystem,-4.3 , "cause i can"))
            )
            
            )
        ));
    }
    
    
}
