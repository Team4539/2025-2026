package frc.robot.commands.AuomaticCommands.ScoringPositions;

import javax.naming.PartialResultException;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.New.drive;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmNeedsCoral;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
            new ParallelCommandGroup(
                new SetElevatorTo(elevatorSubsystem, 4.305419921875),
                new SetCarrigeTo(carrigeSubsystem, 1.27001953125, "L4")).withTimeout(1),
            new ParallelCommandGroup(   // Coral L4 - Initial setup
                new SetElevatorTo(elevatorSubsystem, 4.305419921875),
                new SetCarrigeTo(carrigeSubsystem, 1.27001953125, "cause"),
                new SetArmTo(armRotationSubsystem, 60.0, "coral L4", false)

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
            CommandSwerveDrivetrain m_SwerveDrivetrain) {

        return new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L4 - Positioning and score than run back to a safe position
                new SetArmTo(armRotationSubsystem, 51.5, "coral L4", false),
                new SetElevatorTo(elevatorSubsystem, 4.478271484375)
            ).withTimeout(.7),
        new SetCarrigeTo(carrigeSubsystem, 1.14599609375, null).withTimeout(.5),
        new ParallelRaceGroup(
            new RunHeadManip(headManipSubsystem, -1).withTimeout(1),
            new drive(0, .5, 0, m_SwerveDrivetrain).withTimeout(1))
        );
            
    }
}
