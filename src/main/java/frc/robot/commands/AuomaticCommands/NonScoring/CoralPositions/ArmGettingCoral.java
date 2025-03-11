package frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Elevator;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.commands.BaseCommands.RunIntake;
import frc.robot.commands.BaseCommands.SetElevator;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmGettingCoral extends Command{
    public static Command Handoff(
        ElevatorSubsystem m_ElevatorSubsystem,
        CarrigeSubsystem m_CarriageSubsystem,
        ArmRotationSubsytem m_ArmRotationSubsystem,
        HeadintakeManipulator m_HeadIntakeManipulator,
        IntakeSubsystem m_IntakeSubsystem 
    )
    {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmTo(m_ArmRotationSubsystem, 12, "Handoff", false),
                new SetCarrigeTo(m_CarriageSubsystem, 0.961669921875, "Handoff"),
                new SetElevatorTo(m_ElevatorSubsystem, 1)).withTimeout(1.5),
            new ParallelCommandGroup(
                new RunHeadManip(m_HeadIntakeManipulator, 1),
                new RunIntake(m_IntakeSubsystem, -1, "Handoff")
            ).withTimeout(1),
            new ParallelCommandGroup(
            new ArmHasCoral().ArmupCommand(m_ElevatorSubsystem, m_CarriageSubsystem, m_ArmRotationSubsystem).withTimeout(2),
            new RunIntake(m_IntakeSubsystem, -1, "handoff").withTimeout(1),
            new RunHeadManip(m_HeadIntakeManipulator, 1).withTimeout(1)
        ));
    }
}
