package frc.robot.commands.AuomaticCommands.NonScoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.commands.BaseCommands.RunIntake;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadintakeManipulator;
import frc.robot.subsystems.IntakeSubsystem;

public class Defense extends Command {

    public static Command Handoff(
        ElevatorSubsystem m_ElevatorSubsystem,
        CarrigeSubsystem m_CarriageSubsystem,
        ArmRotationSubsytem m_ArmRotationSubsystem,
        HeadintakeManipulator m_HeadIntakeManipulator,
        IntakeSubsystem m_IntakeSubsystem 
    )
    {
        SmartDashboard.putBoolean("Crazy mode", true);
        return new SequentialCommandGroup(
            new SetArmTo(m_ArmRotationSubsystem, 64, "Handoff", false).withTimeout(1),
            new ParallelCommandGroup(
                new SetArmTo(m_ArmRotationSubsystem, 64, "Handoff", false),
                new SetCarrigeTo(m_CarriageSubsystem, 0, "Handoff"),
                new SetElevatorTo(m_ElevatorSubsystem, 0).withTimeout(2)));
    }
}