package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetElevatorTo;
import frc.robot.commands.SetHeadRotation;
import frc.robot.commands.SetHeadTo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeManipulaotrSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HeadRotationSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> m_chooser;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController Driver = new XboxController(0);
    private final XboxController Operator = new XboxController(1);
    private final Joystick testJoystick = new Joystick(2);
    private final POVButton pov = new POVButton(testJoystick, 0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    

    // driver buttons
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // co-driver buttons
    private final JoystickButton coralL4 = new JoystickButton(Operator, XboxController.Button.kY.value);
    private final JoystickButton coralL3 = new JoystickButton(Operator, XboxController.Button.kX.value);
    private final JoystickButton coralL2 = new JoystickButton(Operator, XboxController.Button.kB.value);
    private final JoystickButton home = new JoystickButton(Operator, XboxController.Button.kA.value);

    //Test Joystick Buttons
    private final JoystickButton selectElecatorCommand = new JoystickButton(testJoystick, 8); // 1 is the trigger button
    private final JoystickButton outTakeCoral = new JoystickButton(testJoystick, 1); // 2 is the thumb button
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final HeadRotationSubsystem m_HeadRotationSubsystem = new HeadRotationSubsystem();
    private final AlgaeManipulaotrSubsystem m_AlgeaManipulatorSubsystem = new AlgaeManipulaotrSubsystem();
    private final CoralManipulatorSubsystem m_CoralManipulatorSubsystem = new CoralManipulatorSubsystem();
    NamedCommands commands = new NamedCommands();


    

    private void configureBindings() {
       
        m_ElevatorSubsystem.setDefaultCommand(
            new SetElevator(() -> Operator.getRawAxis(XboxController.Axis.kLeftY.value), m_ElevatorSubsystem));
        m_HeadRotationSubsystem.setDefaultCommand(
            new SetHeadRotation(() -> Operator.getRawAxis(XboxController.Axis.kRightY.value), m_HeadRotationSubsystem));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        coralL4.whileTrue(new ParallelCommandGroup(new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.coralL4Position, "Coral L4", false), new SetHeadTo(m_HeadRotationSubsystem, Constants.HeadRotator.coralL4Rotation, "Coral L4", false) ));
        coralL3.whileTrue(new ParallelCommandGroup(new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.coralL3Position, "Coral L3", false), new SetHeadTo(m_HeadRotationSubsystem, Constants.HeadRotator.coralReefAngledRotation, "Coral L3", false)));
        coralL2.whileTrue(new ParallelCommandGroup(new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.coralL2Position, "Coral L2", false), new SetHeadTo(m_HeadRotationSubsystem, Constants.HeadRotator.coralReefAngledRotation, "Coral L2", false)));
        home.whileTrue(new ParallelCommandGroup(new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.HomePosition, "Home", false), new SetHeadTo(m_HeadRotationSubsystem, Constants.HeadRotator.HomeRotation, "Home", false)));
        selectElecatorCommand.whileTrue(new ParallelCommandGroup(new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.HomePosition, "Coral L4", true ), new SetHeadTo(m_HeadRotationSubsystem, Constants.HeadRotator.coralL4Rotation, "Coral L4", true)));
        }{}
        
        
        public RobotContainer() {
            m_chooser = AutoBuilder.buildAutoChooser("default");
            SmartDashboard.putData("Auto mode", m_chooser);
            
            NamedCommands.registerCommand("GoL4", new SetElevatorTo(m_ElevatorSubsystem, Constants.Elevator.coralL4Position, "Coral L4", false).withTimeout(3));
            configureBindings();
        }
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}