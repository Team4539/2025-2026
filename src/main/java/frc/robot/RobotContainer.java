package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AuomaticCommands.AlignReef;
import frc.robot.commands.BaseCommands.SetElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> m_chooser;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController Driver = new XboxController(0);
    private final XboxController Operator = new XboxController(1);
    //private final Joystick testJoystick = new Joystick(2);
    //private final POVButton pov = new POVButton(testJoystick, 0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // driver buttons
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final JoystickButton resetGyro = new JoystickButton(Driver, XboxController.Button.kY.value);

    // co-driver buttons
    private final JoystickButton coralL4 = new JoystickButton(Operator, XboxController.Button.kY.value);
    private final JoystickButton coralL3 = new JoystickButton(Operator, XboxController.Button.kX.value);
    private final JoystickButton coralL2 = new JoystickButton(Operator, XboxController.Button.kB.value);
    private final JoystickButton home = new JoystickButton(Operator, XboxController.Button.kA.value);
    private final JoystickButton IntakeAlgae = new JoystickButton(Operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton OutTakeAlgae = new JoystickButton(Operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CoralStation = new JoystickButton(Operator, XboxController.Button.kStart.value);
    private final JoystickButton AlignReef = new JoystickButton(Operator, XboxController.Button.kBack.value);

    // Test Joystick Buttons
    // private final JoystickButton selectElecatorCommand = new JoystickButton(testJoystick, 8); // 1 is the trigger button
    // private final JoystickButton outTakeCoral = new JoystickButton(testJoystick, 1); // 2 is the thumb button
    
    
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    NamedCommands commands = new NamedCommands();

    private void configureBindings() {
        m_ElevatorSubsystem.setDefaultCommand(
            new SetElevator(() -> Operator.getRawAxis(XboxController.Axis.kLeftY.value), m_ElevatorSubsystem)
        );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        resetGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        AlignReef.whileTrue(new AlignReef(5, drivetrain));
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