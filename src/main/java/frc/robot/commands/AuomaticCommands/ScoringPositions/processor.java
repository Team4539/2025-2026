package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class processor extends Command {
    
    public static Command getOnTrueCommand(
            ElevatorSubsystem elevatorSubsystem,
            CarrigeSubsystem carrigeSubsystem,
            ArmRotationSubsytem armRotationSubsystem) {
        SmartDashboard.putBoolean("CoralL4", true);
        
        return 
            new ParallelCommandGroup(   // Coral L4 - Positioning
                new SetArmTo(armRotationSubsystem, 36.7, "coral L4", false),
                new SetCarrigeTo(carrigeSubsystem, 2.310791015625, null),
                new SetElevatorTo(elevatorSubsystem, 0)
            ).withTimeout(2);
            
    
    }
}
