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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Elevator;
import frc.robot.commands.AuomaticCommands.AlignReef;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.BaseCommands.RotateIntake;
import frc.robot.commands.BaseCommands.RunHeadManip;
import frc.robot.commands.BaseCommands.RunIntake;
import frc.robot.commands.BaseCommands.SetArm;
import frc.robot.commands.BaseCommands.SetCarrige;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> m_chooser;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController Driver = new XboxController(0);
    //private final XboxController Operator = new XboxController(1);
    private final Joystick ButtonBox = new Joystick(3);

    //private final Joystick testJoystick = new Joystick(2);
    //private final POVButton pov = new POVButton(testJoystick, 0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // driver buttons
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final JoystickButton resetGyro = new JoystickButton(Driver, XboxController.Button.kY.value);

    // co-driver buttons
    // private final JoystickButton coralL4 = new JoystickButton(Operator, XboxController.Button.kY.value);
    // private final JoystickButton coralL3 = new JoystickButton(Operator, XboxController.Button.kX.value);
    // private final JoystickButton coralL2 = new JoystickButton(Operator, XboxController.Button.kB.value);
    // private final JoystickButton home = new JoystickButton(Operator, XboxController.Button.kA.value);
    // private final JoystickButton IntakeAlgae = new JoystickButton(Operator, XboxController.Button.kRightBumper.value);
    // private final JoystickButton OutTakeAlgae = new JoystickButton(Operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton CoralStation = new JoystickButton(Operator, XboxController.Button.kStart.value);
    // private final JoystickButton AlignReef = new JoystickButton(Operator, XboxController.Button.kBack.value);


    /*Button Box Buttons */

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
    
    // Test Joystick Buttons
    // private final JoystickButton selectElecatorCommand = new JoystickButton(testJoystick, 8); // 1 is the trigger button
    // private final JoystickButton outTakeCoral = new JoystickButton(testJoystick, 1); // 2 is the thumb button
    
    
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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
        // m_ElevatorSubsystem.setDefaultCommand(
        //     new SetElevator(() -> Operator.getRawAxis(XboxController.Axis.kLeftY.value), m_ElevatorSubsystem)
        // );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // m_ArmRotationSubsystem.setDefaultCommand(
        //     new SetArmTo(m_ArmRotationSubsystem, Constants.ArmRotator.HomeRotation, "Home", false)
        // );
        // m_CarrigeSubsystem.setDefaultCommand(
        //     new SetCarrigeTo(m_CarrigeSubsystem, Constants.Carrige.HomePosition, "Home", false)
        // );
        // m_ElevatorSubsystem.setDefaultCommand(
        //     new SetElevatorTo(m_ElevatorSubsystem, Elevator.HomePosition)
        // );

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

        
        //AlignReef.whileTrue(new AlignReef(5, drivetrain));

        /*Joystick Buttons */
        ElevatorUp.whileTrue(new SetElevator(-0.4, m_ElevatorSubsystem));
        ElevatorDown.whileTrue(new SetElevator(0.4, m_ElevatorSubsystem));
        CarrigeUp.whileTrue(new SetCarrige(m_CarrigeSubsystem, -0.4, "Button"));
        CarrigeDown.whileTrue(new SetCarrige(m_CarrigeSubsystem, 0.4, "Button"));
        ArmUp.whileTrue(new SetArm(m_ArmRotationSubsystem, .6, "Button"));
        ArmDown.whileTrue(new SetArm(m_ArmRotationSubsystem, -.6, "Button"));
        HeadIntake.whileTrue(new ParallelCommandGroup(new RunHeadManip(m_headManip, -0.5), new RunIntake(m_intakeSubsytem, -1, "Button")));
        HeadOuttake.whileTrue(new ParallelCommandGroup(new RunHeadManip(m_headManip, 0.5), new RunIntake(m_intakeSubsytem, 10, "button")));
        IntakeRotateIn.whileTrue(new RotateIntake(m_intakeSubsytem, 0.4, "Button"));
        IntakeRotateOut.whileTrue(new RotateIntake(m_intakeSubsytem, -0.4, "Button"));
        TestButton1.whileTrue(new ParallelCommandGroup(
            new SetElevatorTo(m_ElevatorSubsystem, 2.689697265625),
            new SetCarrigeTo(m_CarrigeSubsystem, 2.469482421875, "cause i can"),
            new SetArmTo(m_ArmRotationSubsystem, 16, "Home", false)
        ));

        


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