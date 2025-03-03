package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import javax.naming.PartialResultException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Elevator;
import frc.robot.commands.AuomaticCommands.AlignReef;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL4;
import frc.robot.commands.BaseCommands.RotateIntake;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.commands.BaseCommands.RunIntake;
import frc.robot.commands.BaseCommands.SetArm;
import frc.robot.commands.BaseCommands.SetCarrige;
import frc.robot.commands.BaseCommands.SetClimber;
import frc.robot.commands.BaseCommands.SetElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CarrigeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.HeadintakeManipulator;

public class RobotContainer {
    // Speed constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> m_chooser;

    // Controllers
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController Driver = new XboxController(0);
    private final XboxController Operator = new XboxController(1);
    private final Joystick ButtonBox = new Joystick(3);
    
    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Driver buttons
    private final JoystickButton resetGyro = new JoystickButton(Driver, XboxController.Button.kY.value);
    private final JoystickButton AntiTipper = new JoystickButton(Driver, XboxController.Button.kRightBumper.value);

    // Co-driver buttons (commented out)
    // private final JoystickButton coralL4 = new JoystickButton(Operator, XboxController.Button.kY.value);
    // private final JoystickButton coralL3 = new JoystickButton(Operator, XboxController.Button.kX.value);
    // private final JoystickButton coralL2 = new JoystickButton(Operator, XboxController.Button.kB.value);
    // private final JoystickButton home = new JoystickButton(Operator, XboxController.Button.kA.value);
    // private final JoystickButton IntakeAlgae = new JoystickButton(Operator, XboxController.Button.kRightBumper.value);
    // private final JoystickButton OutTakeAlgae = new JoystickButton(Operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton CoralStation = new JoystickButton(Operator, XboxController.Button.kStart.value);
    // private final JoystickButton AlignReef = new JoystickButton(Operator, XboxController.Button.kBack.value);

    // Button Box Buttons
    private final JoystickButton ElevatorUp = new JoystickButton(ButtonBox, 1);
    private final JoystickButton ElevatorDown = new JoystickButton(ButtonBox, 2);
    private final JoystickButton CarrigeUp = new JoystickButton(ButtonBox, 3);
    private final JoystickButton CarrigeDown = new JoystickButton(ButtonBox, 4);
    private final JoystickButton ArmUp = new JoystickButton(ButtonBox, 6);
    private final JoystickButton ArmDown = new JoystickButton(ButtonBox, 5);
    private final JoystickButton HeadIntake = new JoystickButton(ButtonBox, 7);
    private final JoystickButton HeadOuttake = new JoystickButton(ButtonBox, 8);
    private final JoystickButton IntakeRotateIn = new JoystickButton(ButtonBox, 9);
    private final JoystickButton IntakeRotateOut = new JoystickButton(ButtonBox, 10);
    private final JoystickButton TestButton1 = new JoystickButton(ButtonBox, 11);
    private final JoystickButton TestButton2 = new JoystickButton(ButtonBox, 12);
    
    // Test Joystick Buttons (commented out)
    // private final JoystickButton selectElecatorCommand = new JoystickButton(testJoystick, 8); // 1 is the trigger button
    // private final JoystickButton outTakeCoral = new JoystickButton(testJoystick, 1); // 2 is the thumb button

    /* Subsystems */
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CarrigeSubsystem m_CarrigeSubsystem = new CarrigeSubsystem();
    private final ArmRotationSubsytem m_ArmRotationSubsystem = new ArmRotationSubsytem();
    private final IntakeSubsystem m_intakeSubsytem = new IntakeSubsystem();
    private final HeadintakeManipulator m_headManip = new HeadintakeManipulator();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    NamedCommands commands = new NamedCommands();
    
    private void configureBindings() {

        // Drivetrain default command - execute periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)      // Drive forward with negative Y
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)       // Drive left with negative X
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
            )
        );

        // Anti-tipper - drive at half speed
        AntiTipper.whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/4)      // Half speed forward
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed/4)       // Half speed left
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate/4) // Half angular rate
            )
        );

        // Joystick brake and point controls
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // SysId routines when holding back/start and X/Y
        // Each routine should be run exactly once in a single log
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        resetGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Telemetry registration
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Align with reef (commented out)
        // AlignReef.whileTrue(new AlignReef(5, drivetrain));

        /* Button Box Controls */
        
        // Elevator controls
        ElevatorUp.whileTrue(new SetElevator(-0.4, m_ElevatorSubsystem));
        ElevatorDown.whileTrue(new SetElevator(0.4, m_ElevatorSubsystem));
        
        // Carriage controls
        CarrigeUp.whileTrue(new SetCarrige(m_CarrigeSubsystem, -0.4, "Button"));
        CarrigeDown.whileTrue(new SetCarrige(m_CarrigeSubsystem, 0.4, "Button"));
        
        // Arm controls
        ArmUp.whileTrue(new SetArm(m_ArmRotationSubsystem, .6, "Button"));
        ArmDown.whileTrue(new SetArm(m_ArmRotationSubsystem, -.6, "Button"));
        
        // Head intake/outtake
        HeadIntake.whileTrue(new ParallelCommandGroup(
            new RunHeadManip(m_headManip, -0.5),
            new RunIntake(m_intakeSubsytem, -1, "Button")
        ));
        HeadOuttake.whileTrue(new ParallelCommandGroup(
            new RunHeadManip(m_headManip, 0.5),
            new RunIntake(m_intakeSubsytem, 1, "button")
        ));
        
        // Intake rotation
        IntakeRotateIn.whileTrue(new RotateIntake(m_intakeSubsytem, 0.4, "Button"));
        IntakeRotateOut.whileTrue(new RotateIntake(m_intakeSubsytem, -0.4, "Button"));
        
        /* Arm Staging Commands */
        
        // TestButton1 - Arm staging to clear reef
        TestButton1.onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(   // Arm staging to clear the reef while moving
                new SetElevatorTo(m_ElevatorSubsystem, 1.087158203125),
                new SetCarrigeTo(m_CarrigeSubsystem, 0, "cause i can"),
                new SetArmTo(m_ArmRotationSubsystem, 60, "Home", false)
            ).withTimeout(5)
        ));

        // TestButton1 - Pass coral to head (commented out)
        // TestButton1.whileTrue(new ParallelCommandGroup(   // To pass off the coral to the head (Stage 2 intake process)
        //     new SetElevatorTo(m_ElevatorSubsystem, 2.689697265625),
        //     new SetCarrigeTo(m_CarrigeSubsystem, 2.469482421875, "cause i can"),
        //     new SetArmTo(m_ArmRotationSubsystem, 15, "Home", false)
        // ));
        
        // TestButton2 - Allow intake to move (commented out)
        // TestButton2.onTrue(new ParallelCommandGroup(   // To Allow the Intake to move (Stage 1 intake process)
        //     new SetElevatorTo(m_ElevatorSubsystem, 2.919677734375),
        //     new SetCarrigeTo(m_CarrigeSubsystem, 0, "cause i can"),
        //     new SetArmTo(m_ArmRotationSubsystem, 14, "Home", false)
        // ).withTimeout(5));
        
        // TestButton2/1 - Climber controls (commented out)
        // TestButton2.whileTrue(
        //     new SetClimber(m_climberSubsystem, 1)
        // );
        // TestButton1.whileTrue(
        //     new SetClimber(m_climberSubsystem, -1)
        // );

        // /* Coral L4 */

        TestButton2.whileTrue(CoralL4.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));
        TestButton2.onFalse(CoralL4.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));
        
        /* Coral L3 (commented out) */
        TestButton2.onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(   // Coral L3 - Initial setup
                new SetElevatorTo(m_ElevatorSubsystem, 0),
                new SetCarrigeTo(m_CarrigeSubsystem, 2.337646484375, "cause")
            ).withTimeout(3),
            new ParallelCommandGroup(   // Coral L3 - Positioning
                new SetElevatorTo(m_ElevatorSubsystem, 0),
                new SetCarrigeTo(m_CarrigeSubsystem, 2.337646484375, "cause i can"),
                new SetArmTo(m_ArmRotationSubsystem, 48.0, "coral L4", false)
            ).withTimeout(1),
            new ParallelCommandGroup(   // Coral L3 - Output
                new SetElevatorTo(m_ElevatorSubsystem, 0),
                new SetCarrigeTo(m_CarrigeSubsystem, 3.04833984375, "cause i can"),
                new SetArmTo(m_ArmRotationSubsystem, 60, "coral L4", false),
                new RunHeadManip(m_headManip, .5)
            ).withTimeout(.5),
            new ParallelCommandGroup(   // Return to safe position
                new SetElevatorTo(m_ElevatorSubsystem, 1.087158203125),
                new SetCarrigeTo(m_CarrigeSubsystem, 0, "cause i can"),
                new SetArmTo(m_ArmRotationSubsystem, 60, "Home", false)
            ).withTimeout(5)
        ));
        
        /* Coral L2 (commented out) */
        // TestButton2.onTrue(new SequentialCommandGroup(
        //     new ParallelCommandGroup(   // Coral L2 - Initial setup
        //         new SetElevatorTo(m_ElevatorSubsystem, 0),
        //         new SetCarrigeTo(m_CarrigeSubsystem, 4.295654296875, "cause"),
        //         new SetArmTo(m_ArmRotationSubsystem, 60, "hold", false)
        //     ).withTimeout(3),
        //     new ParallelCommandGroup(   // Coral L2 - Positioning
        //         new SetElevatorTo(m_ElevatorSubsystem, 0),
        //         new SetCarrigeTo(m_CarrigeSubsystem, 4.295654296875, "cause i can"),
        //         new SetArmTo(m_ArmRotationSubsystem, 45.3, "coral L4", false)
        //     ).withTimeout(1),
        //     // new ParallelCommandGroup(  // Finalization (commented out)
        //     //     new SetArmTo(m_ArmRotationSubsystem, 41.6, "Finalizing", false), 
        //     //     new RunHeadManip(m_headManip, .5)
        //     // ).withTimeout(.5),
        //     new ParallelCommandGroup(   // Return to intake position
        //         new SetElevatorTo(m_ElevatorSubsystem, 2.919677734375),
        //         new SetCarrigeTo(m_CarrigeSubsystem, 0, "cause i can"),
        //         new SetArmTo(m_ArmRotationSubsystem, 14, "coral L4", false),
        //         new RunHeadManip(m_headManip, .5)
        //     ).withTimeout(3)
        // ));
    }

    public RobotContainer() {
        m_chooser = AutoBuilder.buildAutoChooser("default");
        SmartDashboard.putData("Auto mode", m_chooser);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}