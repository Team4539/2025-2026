package frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands;

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

public class L1AlgaePickup extends Command {
    
    public static Command onTrueCommand(
        ElevatorSubsystem m_ElevatorSubsystem,
        ArmRotationSubsytem m_ArmRotationSubsytem,
        CarrigeSubsystem m_CarrigeSubsystem, 
        HeadintakeManipulator manipulator
        
    ){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetCarrigeTo(m_CarrigeSubsystem, 0.079833984375, "A2"),
                new SetElevatorTo(m_ElevatorSubsystem, 1.84228515625)
            ).withTimeout(2),
            new ParallelCommandGroup(
                new SetArmTo(m_ArmRotationSubsytem, 36.7, "A2", false),
                new SetCarrigeTo(m_CarrigeSubsystem, 0.079833984375, "A2"),
                new SetElevatorTo(m_ElevatorSubsystem, 1.84228515625),
                new RunHeadManip(manipulator, 1)
            )
        );
}
}
