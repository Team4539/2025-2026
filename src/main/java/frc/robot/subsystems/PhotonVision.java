package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    
    /**
     * Creates a new PhotonVision subsystem for AprilTag detection.
     * @param cameraName The name of the PhotonVision camera
     */
    public PhotonVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
        // Set to AprilTag pipeline if you have multiple pipelines
        camera.setPipelineIndex(0);
        // Optionally turn on the LEDs
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // Get the latest pipeline result
        latestResult = camera.getLatestResult();
        
        // Put basic information on SmartDashboard
        SmartDashboard.putBoolean("AprilTag/Has Targets", hasTargets());
        
        if (latestResult != null) {
            // Fix: Use putNumber instead of getNumber for latency
            //SmartDashboard.putNumber("AprilTag/Pipeline Latency (ms)", latestResult.getLatencyMillis());
            
            if (latestResult.hasTargets()) {
                // Get list of targets
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                SmartDashboard.putNumber("AprilTag/Target Count", targets.size());
                
                // Display information for best target
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
                if (bestTarget != null) {
                    displayTargetInfo("BestTarget", bestTarget);
                }
                
                // Display information for all targets
                for (int i = 0; i < targets.size(); i++) {
                    PhotonTrackedTarget target = targets.get(i);
                    if (target != null) {
                        displayTargetInfo("Target_" + i, target);
                    }
                }
            } else {
                SmartDashboard.putNumber("AprilTag/Target Count", 0);
            }
        } else {
            // Handle case where latestResult is null
            SmartDashboard.putNumber("AprilTag/Pipeline Latency (ms)", -1);
            SmartDashboard.putNumber("AprilTag/Target Count", 0);
            SmartDashboard.putBoolean("AprilTag/Camera Connected", false);
        }
        
        // Always update camera connection status
        SmartDashboard.putBoolean("AprilTag/Camera Connected", camera.isConnected());
    }
    
    /**
     * Displays all available information about a target on SmartDashboard
     * @param prefix The prefix to use for SmartDashboard keys
     * @param target The target to display information for
     */
    private void displayTargetInfo(String prefix, PhotonTrackedTarget target) {
        if (target == null) return;
        
        // Basic target information
        SmartDashboard.putNumber("AprilTag/" + prefix + "/ID", target.getFiducialId());
        SmartDashboard.putNumber("AprilTag/" + prefix + "/Yaw", target.getYaw());
        SmartDashboard.putNumber("AprilTag/" + prefix + "/Pitch", target.getPitch());
        SmartDashboard.putNumber("AprilTag/" + prefix + "/Area", target.getArea());
        SmartDashboard.putNumber("AprilTag/" + prefix + "/Skew", target.getSkew());
        
        // 3D position (in camera space)
        var pose = target.getBestCameraToTarget();
        if (pose != null) {
            SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseX", pose.getX());
            SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseY", pose.getY());
            //SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseZ", pose.getZ());
            
            if (pose.getRotation() != null) {
                SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseRoll", pose.getRotation().getX());
                SmartDashboard.putNumber("AprilTag/" + prefix + "/PosePitch", pose.getRotation().getY());
                SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseYaw", pose.getRotation().getZ());
            }
        }
        
        // Alternative pose solution
        // var altPose = target.getAlternateCameraToTarget();
        // if (altPose != null) {
        //     SmartDashboard.putNumber("AprilTag/" + prefix + "/AltPoseX", altPose.getX());
        //     SmartDashboard.putNumber("AprilTag/" + prefix + "/AltPoseY", altPose.getY());
        //     SmartDashboard.putNumber("AprilTag/" + prefix + "/AltPoseZ", altPose.getZ());
        // }
        
        // // Ambiguity information
        // SmartDashboard.putNumber("AprilTag/" + prefix + "/PoseAmbiguity", target.getPoseAmbiguity());
        
        // Corner information
        var corners = target.getDetectedCorners();
        if (corners != null) {
            for (int i = 0; i < corners.size(); i++) {
                if (corners.get(i) != null) {
                    SmartDashboard.putNumber("AprilTag/" + prefix + "/Corner" + i + "X", corners.get(i).x);
                    SmartDashboard.putNumber("AprilTag/" + prefix + "/Corner" + i + "Y", corners.get(i).y);
                }
            }
        }
    }
    
    /**
     * @return The latest pipeline result from the camera
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }
    
    /**
     * @return Whether the camera currently sees any targets
     */
    public boolean hasTargets() {
        return latestResult != null && latestResult.hasTargets();
    }
    
    /**
     * @return The best target from the latest result, or null if no targets
     */
    public PhotonTrackedTarget getBestTarget() {
        if (hasTargets()) {
            return latestResult.getBestTarget();
        }
        return null;
    }
    
    /**
     * @return All detected targets from the latest result
     */
    public List<PhotonTrackedTarget> getAllTargets() {
        if (hasTargets()) {
            return latestResult.getTargets();
        }
        return List.of();
    }
    
    /**
     * @return Whether the camera is connected
     */
    public boolean isCameraConnected() {
        return camera.isConnected();
    }
}