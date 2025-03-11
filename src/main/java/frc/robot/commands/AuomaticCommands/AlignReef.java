package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class AlignReef extends Command {
    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private CommandSwerveDrivetrain m_drive;
    private int m_tag;
    private PhotonVision m_vision;
    
    // PID Controllers for alignment
    private final PIDController rotationPID;
    private final PIDController strafePID;
    private final PIDController distancePID;
    
    // Target alignment values
    private double targetArea = 4.4097656249999995; // Default target area value (0-100)
    private double xSetpoint = 4.4097656249999995;  // Target horizontal position (0 = centered)
    private double rotSetpoint = 0.0; // Target rotation angle (0 = straight)
    
    // Status tracking
    private boolean tagVisible = false;
    private double currentX = 0.0;
    private double currentArea = 0.0;
    private double currentRotation = 0.0;
    private double lastRawRotation = 0.0;
    private double accumulatedRotation = 0.0; // Tracks continuous rotation
    private boolean firstMeasurement = true;
    
    // Filtering to reduce noise
    private MedianFilter rotationFilter = new MedianFilter(5); // Filter out spikes
    
    // PID tuning parameters
    private double rotationP = 0.03;
    private double rotationI = 0.0;
    private double rotationD = 0.0;
    
    private double strafeP = 0.001;
    private double strafeI = 0.0;
    private double strafeD = 0.0;
    
    private double distanceP = .5;
    private double distanceI = 0.0;
    private double distanceD = 0.0;
    
    // Mode selection
    private boolean useParallelAlignment = true; // Set to true to keep orientation parallel to tag

    /**
     * Creates a command that aligns to an AprilTag with the default settings.
     * 
     * @param tag The AprilTag ID to target
     * @param drive_subsystem The drivetrain to use
     * @param vision The vision subsystem
     */
    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem, PhotonVision vision) {
        this(tag, drive_subsystem, vision, 4.0); // Default target area
    }
    
    /**
     * Creates a command that aligns to an AprilTag and drives to a specific area.
     * 
     * @param tag The AprilTag ID to target
     * @param drive_subsystem The drivetrain to use
     * @param vision The vision subsystem
     * @param targetArea The target area to achieve (0-100)
     */
    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem, PhotonVision vision, double targetArea) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;
        m_vision = vision;
        this.targetArea = targetArea;

        // Configure PID controllers
        rotationPID = new PIDController(rotationP, rotationI, rotationD);
        strafePID = new PIDController(strafeP, strafeI, strafeD);
        distancePID = new PIDController(distanceP, distanceI, distanceD);

        // Set tolerances for when we consider ourselves "aligned"
        rotationPID.setTolerance(5.0);
        strafePID.setTolerance(1.0);
        distancePID.setTolerance(0.5);
        
        // Enable continuous input for rotation PID controller to handle 180/-180 wraparound
        rotationPID.enableContinuousInput(-180, 180);
        
        // Make the command configurable from the dashboard
        SmartDashboard.putData("AlignReef", this);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        strafePID.reset();
        distancePID.reset();
        
        // Set PID setpoints
        rotationPID.setSetpoint(rotSetpoint);
        strafePID.setSetpoint(xSetpoint);
        distancePID.setSetpoint(targetArea);
        
        firstMeasurement = true;
        accumulatedRotation = 0;
        tagVisible = false;
        rotationFilter.reset();
    }

    /**
     * Handles angle wraparound for smooth rotation control
     * @param newAngle The new angle measurement in degrees
     * @return A filtered angle value that handles wraparound
     */
    private double handleAngleWraparound(double newAngle) {
        if (firstMeasurement) {
            lastRawRotation = newAngle;
            accumulatedRotation = newAngle;
            firstMeasurement = false;
            return newAngle;
        }
        
        // Calculate the difference, considering the wraparound
        double diff = newAngle - lastRawRotation;
        
        // Handle wraparound cases
        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }
        
        // Update accumulated rotation
        accumulatedRotation += diff;
        
        // Save the current raw rotation for next comparison
        lastRawRotation = newAngle;
        
        // Apply median filter to smooth out any jumps
        double filtered = rotationFilter.calculate(accumulatedRotation);
        
        // Log values for debugging
        SmartDashboard.putNumber("Target/AccumulatedRotation", accumulatedRotation);
        SmartDashboard.putNumber("Target/FilteredRotation", filtered);
        
        return filtered;
    }
    
    /**
     * Alternative approach: track robot parallel to tag
     * @param yaw The yaw angle to the tag (in degrees)
     * @return Control output for rotation
     */
    private double calculateParallelRotation(double yaw) {
        // For true parallelism, we need to use the tag's rotation rather than its yaw
        // We use the camera's measured rotation value relative to the tag
        return rotationPID.calculate(currentRotation, 0); // We want rotation to be 0 for parallel
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_vision.getLatestResult();
        tagVisible = false;
        
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            
            // Check if we see the correct AprilTag
            if (target.getFiducialId() == m_tag) {
                tagVisible = true;
                
                // Use the 3D pose for more accurate measurements
                var transform3d = target.getBestCameraToTarget();
                Pose3d pose3d = new Pose3d().plus(transform3d);
                
                // Extract target metrics
                currentX = target.getYaw();          // Horizontal offset in degrees
                currentArea = target.getArea();      // Target area as percentage
                
                // For true parallelism, we use the skew of the tag
                // The skew will be 0 when we're parallel to the tag
                currentRotation = target.getSkew();
                
                // Log data for debugging
                SmartDashboard.putNumber("Target/X", currentX);
                SmartDashboard.putNumber("Target/Area", currentArea);
                SmartDashboard.putNumber("Target/Skew", currentRotation);
                
                // Calculate control outputs
                double rotationOutput = rotationPID.calculate(currentRotation, 0); // Target skew of 0 for parallel
                double strafeOutput = strafePID.calculate(currentX);
                double forwardOutput = distancePID.calculate(currentArea);
                
                // Apply speed limits
                rotationOutput = Math.max(-Math.PI, Math.min(Math.PI, rotationOutput));
                strafeOutput = Math.max(-0.5, Math.min(0.5, strafeOutput));
                forwardOutput = Math.max(-0.5, Math.min(0.5, forwardOutput));

                // Apply combined movement
                m_drive.setControl(drive
                     .withVelocityY(-forwardOutput)  // Forward/backward based on area
                     .withVelocityX(strafeOutput) // Left/right based on X
                     .withRotationalRate(-rotationOutput)); // Rotation based on skew
            }
        } else {
            // If we don't see the tag, stop
            stopMovement();
        }
        
        // Update SmartDashboard
        SmartDashboard.putBoolean("Target/HasTarget", tagVisible);
        SmartDashboard.putBoolean("Target/AtSetpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Target/ParallelMode", useParallelAlignment);
    }

    private boolean isAtSetpoint() {
        return strafePID.atSetpoint() && distancePID.atSetpoint() && 
               (useParallelAlignment ? Math.abs(currentX - xSetpoint) < 3.0 : rotationPID.atSetpoint());
    }

    private void stopMovement() {
        m_drive.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we're aligned with the target
        return isAtSetpoint();
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AlignReef");
        
        // Target setpoints
        builder.addDoubleProperty("xSetpoint", () -> this.xSetpoint, (val) -> {
            this.xSetpoint = val;
            this.strafePID.setSetpoint(xSetpoint);
        });
        builder.addDoubleProperty("targetArea", () -> this.targetArea, (val) -> {
            this.targetArea = val;
            this.distancePID.setSetpoint(targetArea);
        });
        builder.addDoubleProperty("rotSetpoint", () -> this.rotSetpoint, (val) -> {
            this.rotSetpoint = val;
            this.rotationPID.setSetpoint(rotSetpoint);
        });
        
        // Current values
        builder.addDoubleProperty("currentX", () -> this.currentX, null);
        builder.addDoubleProperty("currentArea", () -> this.currentArea, null);
        builder.addDoubleProperty("currentRotation", () -> this.currentRotation, null);
        builder.addDoubleProperty("accumulatedRotation", () -> this.accumulatedRotation, null);
        
        // Status
        builder.addBooleanProperty("tagVisible", () -> this.tagVisible, null);
        builder.addBooleanProperty("aligned", () -> isAtSetpoint(), null);
        builder.addBooleanProperty("parallelMode", () -> this.useParallelAlignment, (val) -> this.useParallelAlignment = val);
        
        // PID tuning parameters
        builder.addDoubleProperty("rotationP", () -> this.rotationP, (val) -> {
            this.rotationP = val;
            rotationPID.setP(this.rotationP);
        });
        builder.addDoubleProperty("strafeP", () -> this.strafeP, (val) -> {
            this.strafeP = val;
            strafePID.setP(this.strafeP);
        });
        builder.addDoubleProperty("distanceP", () -> this.distanceP, (val) -> {
            this.distanceP = val;
            distancePID.setP(this.distanceP);
        });
    }
}