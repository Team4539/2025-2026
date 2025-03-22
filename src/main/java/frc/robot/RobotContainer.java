package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.security.AlgorithmConstraints;
import java.util.jar.Attributes.Name;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AuomaticCommands.reefAlignRotation;
import frc.robot.commands.AuomaticCommands.New.visionL4;
import frc.robot.commands.AuomaticCommands.RotateIntaketo;
import frc.robot.commands.AuomaticCommands.SetArmTo;
import frc.robot.commands.AuomaticCommands.SetCarrigeTo;
import frc.robot.commands.AuomaticCommands.SetElevatorTo;
import frc.robot.commands.AuomaticCommands.reefAlignHorizontal;
import frc.robot.commands.AuomaticCommands.NonScoring.Defense;
import frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands.L1AlgaePickup;
import frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands.L2Algaegrab;
import frc.robot.commands.AuomaticCommands.NonScoring.AlgaeCommands.ToggleIntake;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmGettingCoral;
import frc.robot.commands.AuomaticCommands.NonScoring.CoralPositions.ArmHasCoral;
import frc.robot.commands.AuomaticCommands.ScoringPositions.Barge;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL1;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL2;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL3;
import frc.robot.commands.AuomaticCommands.ScoringPositions.CoralL4;
import frc.robot.commands.AuomaticCommands.ScoringPositions.processor;
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
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ArmRotationSubsytem;
import frc.robot.subsystems.HeadintakeManipulator;
@SuppressWarnings("unused")

public class RobotContainer {
    // Speed constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> m_chooser;

    // Controllers
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController Driver = new XboxController(0);
    //private final XboxController Operator = new XboxController(1);
    private final Joystick ButtonBox = new Joystick(1);
    
    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Driver buttons
    private final JoystickButton resetGyro = new JoystickButton(Driver, XboxController.Button.kY.value);
    private final JoystickButton AntiTipper = new JoystickButton(Driver, XboxController.Button.kRightBumper.value);
    //private final JoystickButton CoralDestucker = new JoystickButton(Driver, XboxController.Button.kX.value);
    private final JoystickButton AlignColor = new JoystickButton(Driver, XboxController.Button.kB.value);
    private final JoystickButton AlignTAg = new JoystickButton(Driver, XboxController.Button.kStart.value);
    private final JoystickButton AlignReef = new JoystickButton(Driver, XboxController.Button.kBack.value);
    private final JoystickButton Defender = new JoystickButton(Driver, XboxController.Button.kX.value);


//    // operator Xbox buttons
//     private final JoystickButton DecideArmButton = new JoystickButton(Operator, XboxController.Button.kA.value);
//     private final JoystickButton DecideCarrigeButton = new JoystickButton(Operator, XboxController.Button.kB.value);
//     private final JoystickButton DecideElevatorButton = new JoystickButton(Operator, XboxController.Button.kX.value);
//     private final JoystickButton DecideClimberButton = new JoystickButton(Operator, XboxController.Button.kY.value); 

    // Button Box Buttons
    private final JoystickButton ElevatorUp = new JoystickButton(ButtonBox, 2);  // A1
    private final JoystickButton ElevatorDown = new JoystickButton(ButtonBox, 1); // Barge
    private final JoystickButton CarrigeUp = new JoystickButton(ButtonBox, 4); // proc
    private final JoystickButton CarrigeDown = new JoystickButton(ButtonBox, 3); //gtround alg  // now A2
    private final JoystickButton ArmUp = new JoystickButton(ButtonBox, 5); // L3
    private final JoystickButton ArmDown = new JoystickButton(ButtonBox, 6); // L4
    private final JoystickButton HeadIntake = new JoystickButton(ButtonBox, 7); // L2
    private final JoystickButton HeadOuttake = new JoystickButton(ButtonBox, 8); // L1
    private final JoystickButton IntakeRotateIn = new JoystickButton(ButtonBox, 9); // hand
    private final JoystickButton IntakeRotateOut = new JoystickButton(ButtonBox, 10); // intake
    private final JoystickButton TestButton1 = new JoystickButton(ButtonBox, 12); // Left  // in?
    private final JoystickButton TestButton2 = new JoystickButton(ButtonBox, 11); // Right // out?
    private final POVButton alageA2 = new POVButton(ButtonBox, 0); // Up A2

    
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

    private final PhotonVision m_colorVision = new PhotonVision("Arducam_OV9782_USB_Camera");
    private final PhotonCamera m_aprilTagCamera = new PhotonCamera("Global_Shutter_Camera");

    NamedCommands commands = new NamedCommands();

    public Command getTestDriveCommand() {
        return drivetrain.applyRequest(() -> 
            drive.withVelocityX(0.3 * MaxSpeed) // 30% forward speed
                .withVelocityY(0) 
                .withRotationalRate(0));
    }
    
    private void configureBindings() {
        // Remove the NamedCommands registration from here
        // NamedCommands.registerCommand("ArmUp", ArmHasCoral.ArmupCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem).withTimeout(4));

        // Drivetrain default command - execute periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(), 0.01) * MaxSpeed*.5)
                .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), 0.01) * MaxSpeed*.5) // Drive left with negative X (left)
                .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.01) * MaxAngularRate*.5) // Drive counterclockwise with negative X (left)
      ).ignoringDisable(true));

        // Anti-tipper - drive at half speed 
        AntiTipper.whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/4)      // Half speed forward
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed/4)       // Half speed left
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate/4) // Half angular rate
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

       AlignColor.whileTrue(new reefAlignHorizontal(drivetrain, m_colorVision));

       AlignTAg.whileTrue(new reefAlignRotation(drivetrain, m_aprilTagCamera));

        // DecideArmButton.whileTrue(
        //    new SetArm(m_ArmRotationSubsystem, Operator.getLeftY(), "Button")
        // );

        // CoralDestucker.whileTrue(
        //     new RunIntake(m_intakeSubsytem, -.1, "Destuck")
        // );
        // CoralDestucker.onFalse(
        //     new RunIntake(m_intakeSubsytem, .2, "Destruck").withTimeout(.5)
        // );

       // CarrigeUp.whileTrue(getAutonomousCommand())
       CarrigeUp.onTrue(
        processor.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem)
       );

       ElevatorDown.onTrue(
        ArmHasCoral.ArmupCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem)
       );

       AlignReef.whileTrue(Barge.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));

        Defender.onTrue(Defense.Handoff(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip, m_intakeSubsytem));
       // Sam: Try new AlignReef(5, drivetrain, m_photonVision, TARGETAREA) and mess around with the value
      // AlignReef.whileTrue(new AlignReef(5, drivetrain, m_photonVision));
        // Align with reef (commented out)
        // AlignReef.whileTrue(new AlignReef(5, drivetrain));

        /* Button Box Controls */
        
        // Elevator controls but converted to do auton positions as of now

        //ElevatorUp.whileTrue(new AlignReef(5, drivetrain));
        // ElevatorUp.whileTrue(CoralL3.getOnTruecCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));
        // ElevatorUp.onFalse(CoralL3.getOnFalsCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));
        // ElevatorDown.whileTrue(CoralL2.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));
        // ElevatorDown.onFalse(CoralL2.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));
        
        // // Carriage controls
        // CarrigeUp.whileTrue(new SetCarrige(m_CarrigeSubsystem, -0.4, "Button"));
        // CarrigeDown.whileTrue(new SetCarrige(m_CarrigeSubsystem, 0.4, "Button"));
        
        // // Arm controls
        // ArmUp.whileTrue(new SetArm(m_ArmRotationSubsystem, .6, "Button"));
        // ArmDown.whileTrue(new SetArm(m_ArmRotationSubsystem, -.6, "Button"));
        
        // // Head intake/outtake
        // HeadIntake.whileTrue(new ParallelCommandGroup(
        //     new RunHeadManip(m_headManip, 1.2),
        //     new RunIntake(m_intakeSubsytem, -1, "Button")
        // ));
        // HeadOuttake.whileTrue(new ParallelCommandGroup(Q
        //     new RunHeadManip(m_headManip, -1.2),
        //     new RunIntake(m_intakeSubsytem, 1, "button")
        // ));
        
        // // Intake rotation
        IntakeRotateOut.whileTrue(
            new SequentialCommandGroup(
                    new RotateIntaketo(m_intakeSubsytem, .4, "intake out", false).withTimeout(2),
                new RunIntake(m_intakeSubsytem, 1, "Button")
            )
        );
        IntakeRotateOut.onFalse(
                new RotateIntaketo(m_intakeSubsytem, .959, "Back", false).withTimeout(3)
        );
        
        IntakeRotateIn.onTrue(
            ArmGettingCoral.Handoff(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip, m_intakeSubsytem)
        );
        
        ArmDown.whileTrue(
            CoralL4.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem)); // L4
        ArmDown.onFalse(CoralL4.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));

        ArmUp.whileTrue(
            CoralL3.getOnTruecCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem)); // L3
        ArmUp.onFalse(CoralL3.getOnFalsCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));
        HeadIntake.onTrue(
            CoralL2.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem)); // L2
        HeadIntake.onFalse(CoralL2.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));
        HeadOuttake.onTrue(
            CoralL1.OntrueCommadn(m_intakeSubsytem, m_ElevatorSubsystem)); // L1
        HeadOuttake.onFalse(CoralL1.OnFalseCommand(m_intakeSubsytem));
        
        CarrigeDown.whileTrue(L2Algaegrab.onTrueCommand(m_ElevatorSubsystem, m_ArmRotationSubsystem, m_CarrigeSubsystem, m_headManip));
        /* Arm Staging Commands */
        
        ElevatorUp.whileTrue(L1AlgaePickup.onTrueCommand(m_ElevatorSubsystem, m_ArmRotationSubsystem, m_CarrigeSubsystem, m_headManip));

        
        TestButton1.whileTrue(
            new RunHeadManip(m_headManip, -1)
        );
        

        // TestButton1.onTrue(
        //     new SequentialCommandGroup(
        //         new reefAlignRotation(drivetrain, m_aprilTagCamera).withTimeout(5),
        //         Commands.startEnd(
        //             () -> drivetrain.setControl(drive
        //                     .withVelocityY(0.3)
        //                     .withVelocityX(0.0)
        //                     .withRotationalRate(0.0)),
        //             () -> drivetrain.setControl(drive
        //                     .withVelocityY(0.0)
        //                     .withVelocityX(0.0)
        //                     .withRotationalRate(0.0)),
        //             drivetrain
        //         ).withTimeout(0.5),
        //         new reefAlignHorizontal(drivetrain, m_colorVision)
        //         )
        //     );

        TestButton2.whileTrue(
            new RunHeadManip(m_headManip, 1)
        );

    

        // TestButton1 - Arm staging to clear reef
        // TestButton1.onTrue(new SequentialCommandGroup(
        //     new ParallelCommandGroup(   // Arm staging to clear the reef while moving
        //         new SetElevatorTo(m_ElevatorSubsystem, 1.087158203125),
        //         new SetCarrigeTo(m_CarrigeSubsystem,  0, "cause i can"),
        //         new SetArmTo(m_ArmRotationSubsystem, 60, "Home", false)
        //     ).withTimeout(5)
        // ));

        // TestButton2.whileTrue(CoralL4.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));
        // TestButton2.onFalse(CoralL4.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));   

    }
    public RobotContainer() {
        // Register named commands right after subsystem initialization
        // but before auto chooser creation
        registerNamedCommands();
        
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto mode", m_chooser);
        configureBindings();
    }
    
    private void registerNamedCommands() {
        // Register all named commands here

        NamedCommands.registerCommand("L4 Coral Auto", visionL4.run(drivetrain, m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip, m_colorVision, m_aprilTagCamera));
        
        NamedCommands.registerCommand("ArmUp", 
            ArmHasCoral.ArmupCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem).withTimeout(1));
        NamedCommands.registerCommand("CoralL1Start", 
            CoralL1.OntrueCommadn(m_intakeSubsytem, m_ElevatorSubsystem).withTimeout(2));
        NamedCommands.registerCommand("CoralL1Finsih", 
            CoralL1.OnFalseCommand(m_intakeSubsytem));
        NamedCommands.registerCommand("L4Prep", 
            CoralL4.getOnTrueCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem));
        NamedCommands.registerCommand("L4Finish",
            CoralL4.getOnFalseCommand(m_ElevatorSubsystem, m_CarrigeSubsystem, m_ArmRotationSubsystem, m_headManip));

        // Add any other named commands here
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}