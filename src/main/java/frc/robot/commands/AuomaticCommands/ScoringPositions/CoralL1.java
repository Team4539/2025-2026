package frc.robot.commands.AuomaticCommands.ScoringPositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.RotateIntaketo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.BaseCommands.RotateIntake;
import frc.robot.commands.BaseCommands.RunIntake;
import frc.robot.commands.BaseCommands.SetElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CoralL1 extends Command {

    public static SequentialCommandGroup OntrueCommadn(
        IntakeSubsystem m_IntakeSubsystem,
        ElevatorSubsystem m_ElevatorSubsystem
    ) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RotateIntaketo(m_IntakeSubsystem, 0.8, "L1", false),
                new SetElevatorTo(m_ElevatorSubsystem, 3.480224609375)).withTimeout(3),
                

            //new RotateIntake(m_IntakeSubsystem, -.1,  "L1").withTimeout(1), 
            new RunIntake(m_IntakeSubsystem, -.4, "L!"));

    }

    public static SequentialCommandGroup OnFalseCommand(
        IntakeSubsystem m_IntakeSubsystem
    ) {
        return new SequentialCommandGroup(
            new RotateIntaketo(m_IntakeSubsystem, .98, "L1", false).withTimeout(2),
            new RunIntake(m_IntakeSubsystem, 0, "L1")
        );
    }
}
