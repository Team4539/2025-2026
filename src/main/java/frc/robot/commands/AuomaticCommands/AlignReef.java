package frc.robot.commands.AuomaticCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
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
    
    // Parameters for distance driving
    private double targetDistance;
    private boolean driveToDistance = false;
    private double driveSpeed = 0.3; // Speed to drive when approaching the reef
    private long lastTargetTimestamp = 0;
    private static final long TARGET_TIMEOUT_MS = 1000; // Increased time to continue after losing target
    
    // April tag and odometry correlation variables
    private Pose2d lastAprilTagPose = null;
    private Pose2d lastOdometryPose = null;
    private double tagToOdometryRatio = 1.0; // Correction factor between tag and odometry
    private LinearFilter tagCorrectionFilter = LinearFilter.movingAverage(5); // Smooth correction factor
    private double estimatedDistanceToTravel = 0;
    private double distanceTraveledWhenTargetLost = 0;

    // Pose tracking variables
    private Pose2d startingPose;
    private Pose2d targetPose;
    private boolean targetPoseEstablished = false;
    
    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem, PhotonVision vision) {
        this(tag, drive_subsystem, vision, 0); // Default constructor with no distance
    }
    
    public AlignReef(int tag, CommandSwerveDrivetrain drive_subsystem, PhotonVision vision, double distanceMeters) {
        addRequirements(drive_subsystem);
        m_tag = tag;
        m_drive = drive_subsystem;
        m_vision = vision;
        targetDistance = distanceMeters;
        driveToDistance = (distanceMeters > 0);

        // Configure PID controllers with gains
        rotationPID = new PIDController(0.03, 0, 0);
        strafePID = new PIDController(0.5, 0, 0);
        distancePID = new PIDController(1, 0, 0);

        // Set tolerances for when we consider ourselves "aligned"
        rotationPID.setTolerance(1.0);
        strafePID.setTolerance(1.0);
        distancePID.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        strafePID.reset();
        distancePID.reset();
        
        // Record starting position
        startingPose = m_drive.getState().Pose;
        targetPoseEstablished = false;
        targetPose = null;
        
        // Reset tag correlation tracking
        lastAprilTagPose = null;
        lastOdometryPose = null;
        tagToOdometryRatio = 1.0;
        tagCorrectionFilter.reset();
        
        lastTargetTimestamp = System.currentTimeMillis();
        estimatedDistanceToTravel = 0;
        distanceTraveledWhenTargetLost = 0;
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_vision.getLatestResult();
        boolean targetFound = false;
        Pose2d currentPose = m_drive.getState().Pose;

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            
            // Check if we see the correct AprilTag
            if (target.getFiducialId() == m_tag) {
                targetFound = true;
                lastTargetTimestamp = System.currentTimeMillis();
                
                // Get pose information from the target
                var pose = target.getBestCameraToTarget();
                double ty = pose.getY(); // forward/backward
                double tx = pose.getX(); // left/right
                double tYaw = pose.getRotation().getY(); // rotation
                
                // Store the current tag-based pose
                Pose2d newAprilTagPose = new Pose2d(
                    new Translation2d(ty, tx),
                    new Rotation2d(tYaw)
                );
                
                // Update correlation between tag movement and odometry
                if (lastAprilTagPose != null && lastOdometryPose != null) {
                    // Calculate how much the tag position has changed
                    double tagDeltaX = newAprilTagPose.getX() - lastAprilTagPose.getX();
                    double tagDeltaY = newAprilTagPose.getY() - lastAprilTagPose.getY();
                    double tagDistance = Math.sqrt(tagDeltaX*tagDeltaX + tagDeltaY*tagDeltaY);
                    
                    // Calculate how much odometry has changed
                    double odomDeltaX = currentPose.getX() - lastOdometryPose.getX();
                    double odomDeltaY = currentPose.getY() - lastOdometryPose.getY();
                    double odomDistance = Math.sqrt(odomDeltaX*odomDeltaX + odomDeltaY*odomDeltaY);
                    
                    // Update the ratio if we've moved enough to get a meaningful reading
                    if (odomDistance > 0.02) {  // Only update if we've moved at least 2cm
                        double newRatio = tagDistance / odomDistance;
                        if (newRatio > 0.1 && newRatio < 10) { // Sanity check on ratio
                            tagToOdometryRatio = tagCorrectionFilter.calculate(newRatio);
                        }
                    }
                    
                    // If driving to distance, update estimated distance based on tag data
                    if (driveToDistance && targetDistance > 0) {
                        // How much distance remains according to tag
                        estimatedDistanceToTravel = ty * tagToOdometryRatio;
                    }
                }
                
                // Store current poses for next iteration
                lastAprilTagPose = newAprilTagPose;
                lastOdometryPose = currentPose;
                
                // Continuously update the target pose whenever we see the tag
                // Create a transform based on camera data
                Transform2d targetTransform = new Transform2d(
                    new Translation2d(ty, tx),  
                    new Rotation2d(tYaw)
                );
                
                // Apply transform to get target pose
                targetPose = currentPose.transformBy(targetTransform);
                targetPoseEstablished = true;
                
                // Log data for debugging
                SmartDashboard.putNumber("Target/TX", tx);
                SmartDashboard.putNumber("Target/TY", ty);
                SmartDashboard.putNumber("Target/TYaw", tYaw);
                SmartDashboard.putNumber("Target/TagToOdometryRatio", tagToOdometryRatio);
                
                // Calculate control outputs
                double rotationOutput = rotationPID.calculate(tYaw, 3) * 1.5 * Math.PI;
                double strafeOutput = strafePID.calculate(tx, 2);
                
                double forwardOutput;
                if (driveToDistance) {
                    // PID control based on distance to target
                    forwardOutput = distancePID.calculate(ty, targetDistance);
                    // Clamp output to ensure minimum speed
                    if (forwardOutput > 0) {
                        forwardOutput = Math.max(forwardOutput, driveSpeed);
                    } else {
                        forwardOutput = Math.min(forwardOutput, -driveSpeed);
                    }
                } else {
                    forwardOutput = distancePID.calculate(ty, 0);
                }

                // Apply combined movement
                m_drive.setControl(drive
                     .withVelocityX(forwardOutput) // Forward/backward
                     .withVelocityY(-strafeOutput)   // Left/right
                     .withRotationalRate(rotationOutput*-1)); // Rotation
            }
        }
        
        // If target not found but we have an established target pose
        if (!targetFound && targetPoseEstablished) {
            if (System.currentTimeMillis() - lastTargetTimestamp < TARGET_TIMEOUT_MS) {
                // First time we lost the target, record current traveled distance
                if (lastAprilTagPose != null) {
                    distanceTraveledWhenTargetLost = getDistanceTraveled();
                    lastAprilTagPose = null; // Mark that we're now in "lost target" mode
                }
                
                // Calculate how much we should continue based on estimated distance
                double remainingDistance = 0;
                
                if (driveToDistance) {
                    // Calculate remaining distance based on our correlation model
                    double currentTraveledDistance = getDistanceTraveled();
                    double distanceSinceTargetLost = currentTraveledDistance - distanceTraveledWhenTargetLost;
                    remainingDistance = estimatedDistanceToTravel - (distanceSinceTargetLost * tagToOdometryRatio);
                    
                    // If we're past the expected distance, stop
                    if (remainingDistance <= 0) {
                        stopMovement();
                        return;
                    }
                }
                
                // Use odometry to continue navigating toward the target
                // Calculate vector to target
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();
                double distance = Math.sqrt(dx*dx + dy*dy);
                
                // Calculate angle to target in robot-centric frame
                double angleToTarget = Math.atan2(dy, dx) - currentPose.getRotation().getRadians();
                
                // Calculate robot-centric movement commands with correction factor
                double forwardSpeed = driveSpeed * Math.cos(angleToTarget);
                double strafeSpeed = driveSpeed * Math.sin(angleToTarget);
                
                // Calculate rotation to maintain heading toward target
                double targetHeading = Math.atan2(dy, dx);
                double rotationSpeed = rotationPID.calculate(
                    currentPose.getRotation().getRadians(),
                    targetHeading
                );
                
                // Apply movement
                m_drive.setControl(drive
                    .withVelocityX(forwardSpeed)
                    .withVelocityY(strafeSpeed)
                    .withRotationalRate(rotationSpeed));
                    
                // Log data for debugging
                SmartDashboard.putNumber("Target/Distance", distance);
                SmartDashboard.putNumber("Target/RemainingDistance", remainingDistance);
                SmartDashboard.putNumber("Target/AngleToTarget", Math.toDegrees(angleToTarget));
                SmartDashboard.putNumber("Target/DistanceSinceLost", 
                                         getDistanceTraveled() - distanceTraveledWhenTargetLost);
            } else {
                stopMovement();
            }
        } else if (!targetFound) {
            stopMovement();
        }
        
        // Always update distance traveled on SmartDashboard
        SmartDashboard.putNumber("Target/DistanceTraveled", getDistanceTraveled());
        SmartDashboard.putBoolean("Target/HasTarget", targetFound);
        SmartDashboard.putBoolean("Target/PoseEstablished", targetPoseEstablished);
    }

    /**
     * Calculates the distance traveled using drivetrain odometry
     */
    private double getDistanceTraveled() {
        Pose2d currentPose = m_drive.getState().Pose;
        double dx = currentPose.getX() - startingPose.getX();
        double dy = currentPose.getY() - startingPose.getY();
        
        // Calculate Euclidean distance from starting point
        return Math.sqrt(dx*dx + dy*dy);
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
        // Regular alignment mode: finished when aligned
        if (!driveToDistance) {
            return rotationPID.atSetpoint() && 
                   strafePID.atSetpoint() && 
                   distancePID.atSetpoint();
        } 
        // Distance driving mode: finished when distance reached or target lost for too long
        else {
            double currentDistance = getDistanceTraveled();
            
            return (targetDistance > 0 && currentDistance >= targetDistance) || 
                   (!targetPoseEstablished && System.currentTimeMillis() - lastTargetTimestamp > TARGET_TIMEOUT_MS);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}