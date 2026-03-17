    package frc.robot.subsystems;

    import java.lang.StackWalker.Option;
    import java.util.List;
    import java.util.Optional;

    import org.photonvision.EstimatedRobotPose;
    import org.photonvision.PhotonCamera;
    import org.photonvision.PhotonPoseEstimator;
    import org.photonvision.PhotonPoseEstimator.PoseStrategy;
    import org.photonvision.targeting.PhotonPipelineResult;
    import org.photonvision.targeting.PhotonTrackedTarget;

    import edu.wpi.first.apriltag.AprilTagFieldLayout;
    import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
    import edu.wpi.first.math.geometry.Transform3d;
    import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import swervelib.SwerveDrive;
   import java.lang.Math;
    import swervelib.telemetry.SwerveDriveTelemetry;
    public class Vision extends SubsystemBase {
        private Optional<AprilTagFieldLayout> optionalFieldLayout;
        private AprilTagFieldLayout aprilTagFieldLayout;
        private PhotonCamera camera;
        private Transform3d robotToCam;
        private PhotonPoseEstimator photonPoseEstimator;
        private List<PhotonPipelineResult> unreadResults;
        private PhotonPipelineResult latestResult;
       private final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
private final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
private static final double MAX_AMBIGUITY = 300;
private static final double MAX_DISTANCE = 400.0;
private static final double MAX_Z_ERROR = 5;
private static final double MAX_POSE_JUMP = 200;
private static final double MAX_ROTATION_JUMP = Math.PI *4;
public Vision() {
             aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

            camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
            robotToCam = new Transform3d(new Translation3d(0.117, 0.5,0.686), new Rotation3d(0, 0, 0)); // Cam mounted
                                                                                                    // facing // forward,
                                                                                                    // half a meter
                                                                                                    // forward of
                                                                                                    // center, half
                                                                                                    // a meter up
                                                                                                    // from center.
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

                    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                    System.out.println("[Vision] Initialized successfully");
                    System.out.println("[Vision] Initialized successfully");
                    SmartDashboard.putBoolean("Vision/Initialized", true);

        }

        // Construct PhotonPoseEstimator
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            try {
                photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                unreadResults = camera.getAllUnreadResults();
                
                // ADD: Log result count
                int resultCount = unreadResults != null ? unreadResults.size() : 0;
                SmartDashboard.putNumber("Vision/UnreadResultsCount", resultCount);
                SmartDashboard.putBoolean("Vision/CameraConnected", camera.isConnected());
                
                if (unreadResults.isEmpty()) {
                    SmartDashboard.putBoolean("Vision/HasResults", false);
                    SmartDashboard.putString("Vision/Status", "No unread results");
                    return Optional.empty();
                }
                
                latestResult = unreadResults.get(unreadResults.size() - 1);
                
                boolean hasTargets = latestResult != null && latestResult.hasTargets();
                SmartDashboard.putBoolean("Vision/HasTargets", hasTargets);
                
                if (latestResult == null || !latestResult.hasTargets()) {
                    SmartDashboard.putString("Vision/Status", "No targets detected");
                    return Optional.empty();
                }
                
                // ADD: Log detected tag IDs
                int targetCount = latestResult.getTargets().size();
                SmartDashboard.putNumber("Vision/TargetCount", targetCount);
                
                String targetIDs = "";
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    targetIDs += target.getFiducialId() + ",";
                }
                SmartDashboard.putString("Vision/DetectedTagIDs", targetIDs);
                
                Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update(latestResult);
                
                if (estimate.isPresent()) {
                    SmartDashboard.putString("Vision/Status", "Pose estimate generated ✓");
                } else {
                    SmartDashboard.putString("Vision/Status", "Estimator returned empty");
                }
                
                return estimate;
        
            } catch (Exception e) {
                System.err.println("Error in pose generation: " + e.getMessage());
                e.printStackTrace();
                SmartDashboard.putString("Vision/Status", "ERROR: " + e.getMessage());
                return Optional.empty();
            }
        }
        

    private boolean isValidPose(EstimatedRobotPose pose, Pose2d prevOdometryPose) {
        Pose3d estimatedPose3d = pose.estimatedPose;
        Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
        
        // ===== Check 1: Z-Position Sanity Check =====
        // Robot drives on ground (Z ≈ 0). Large Z indicates:
        // - Tag misidentification
        // - Extreme viewing angles causing solver errors
        // - Reflections or partial tag visibility
        double zError = Math.abs(estimatedPose3d.getZ());
        if (zError > MAX_Z_ERROR) {
            return false;
        }

        double maxAmbiguity = pose.targetsUsed.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .max()
            .orElse(0);
            SmartDashboard.putNumber("Vision/MaxAmbiguity", maxAmbiguity);  // ADD THIS

        
        if (maxAmbiguity > MAX_AMBIGUITY) {
            return false;
        }
        double maxDistance = pose.targetsUsed.stream()
        .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
        .max()
        .orElse(0);
    
    if (maxDistance > MAX_DISTANCE) {
        return false;
    }


    //comment out area for get Translation

    double translationDiff = estimatedPose2d.getTranslation()
    .getDistance(prevOdometryPose.getTranslation());

if (translationDiff > MAX_POSE_JUMP) {
    return false;
}

// Comment area below out to not restrict rotation change. 
double rotationDiff = Math.abs(
    estimatedPose2d.getRotation()
        .minus(prevOdometryPose.getRotation())
        .getRadians()
);

if (rotationDiff > MAX_ROTATION_JUMP) {
    return false;
}

return true;

        
    } 
    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose pose) {
        int tagCount = pose.targetsUsed.size();
        
        if (tagCount >= 2) {
            // Multi-tag is inherently more accurate
            // Multiple perspectives constrain the solution geometrically
            // Use base multi-tag standard deviations
            return MULTI_TAG_STD_DEVS;
            
        } else {
            // Single tag: accuracy degrades with distance
            // At 1 meter: Few pixels of error = small position error
            // At 4 meters: Same pixels of error = large position error
            
            double distance = pose.targetsUsed.get(0)
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm();
            
            // Scale uncertainty linearly with distance
            // distance = 1m → multiplier = 1.0x (base uncertainty)
            // distance = 2m → multiplier = 2.0x (twice as uncertain)
            // distance = 4m → multiplier = 4.0x (four times as uncertain)
            double distanceMultiplier = Math.max(1.0, distance);
            
            // Apply scaling to all three dimensions
            return VecBuilder.fill(
                SINGLE_TAG_STD_DEVS.get(0, 0) * distanceMultiplier,  // X uncertainty
                SINGLE_TAG_STD_DEVS.get(1, 0) * distanceMultiplier,  // Y uncertainty
                SINGLE_TAG_STD_DEVS.get(2, 0) * distanceMultiplier   // Rotation uncertainty
            );
        }
    }
    
    public void updatePoseEstimation(SwerveDrive swerveDrive, Pose2d prevEstimatedRobotPose) {
        Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(prevEstimatedRobotPose);
        SmartDashboard.putBoolean("Vision/PoseEstimatePresent", poseEst.isPresent());

        poseEst.ifPresent(pose -> {
            try {
                Pose3d estimatedPose3d = pose.estimatedPose;
                Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
                SmartDashboard.putNumber("Vision/EstimatedPoseX", estimatedPose2d.getX());
                SmartDashboard.putNumber("Vision/EstimatedPoseY", estimatedPose2d.getY());
                SmartDashboard.putNumber("Vision/EstimatedPoseZ", estimatedPose3d.getZ());
                SmartDashboard.putNumber("Vision/EstimatedRotation", estimatedPose2d.getRotation().getDegrees());
                SmartDashboard.putNumber("Vision/Timestamp", pose.timestampSeconds);

                if (!isValidPose(pose, prevEstimatedRobotPose)){
                    return;
                }

                Matrix<N3,N1> stdDevs = calculateStdDevs(pose);
                swerveDrive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),  // Convert estimated pose to Pose2d
                    pose.timestampSeconds,
                    stdDevs       // Timestamp from the vision system
                );  
            } catch (Exception e) {
                System.err.println("Error adding vision measurement: " + e.getMessage());
            }
        });


    
        
    
}
 }