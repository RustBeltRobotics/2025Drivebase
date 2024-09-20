package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;

/**
 * Photonvision based vision system - note we are NOT modeled as a Subsystem, since we don't want to block any commands
 * 
 */
public class VisionSystem {

    private final AprilTagFieldLayout fieldLayout;
    private final List<PhotonCamera> cameras = new ArrayList<>();
    private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> acceptedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Rejected", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> acceptedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Rejected", Pose3d.struct).publish();

    //Links for troubleshooting / understanding:
    /*
     * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/index.html
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html
     * https://www.chiefdelphi.com/t/trouble-with-cameratotarget-and-robot-pose-from-photonvision/454206
     * https://github.com/TexasTorque/TorqueLib/blob/master/sensors/TorqueVision.java
     */

    public VisionSystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        PhotonCamera camera1 = new PhotonCamera(Constants.Vision.CameraName.CAMERA_1);
        PhotonCamera camera2 = new PhotonCamera(Constants.Vision.CameraName.CAMERA_2);
        PhotonCamera camera3 = new PhotonCamera(Constants.Vision.CameraName.CAMERA_3);
        PhotonCamera camera4 = new PhotonCamera(Constants.Vision.CameraName.CAMERA_4);

        cameras.add(camera1);
        cameras.add(camera2);
        cameras.add(camera3);
        cameras.add(camera4);
        //TODO: set pipeline index explicitly for all cameras for AprilTag processing

        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, Constants.Vision.CameraPose.CAMERA_1));
        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, Constants.Vision.CameraPose.CAMERA_2));
        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera3, Constants.Vision.CameraPose.CAMERA_3));
        poseEstimators.add(new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera4, Constants.Vision.CameraPose.CAMERA_4));
    }
 
    public List<EstimatedRobotPose> getRobotPoseEstimation() {
        List<EstimatedRobotPose> acceptedPoses = new ArrayList<>(poseEstimators.size());
        List<EstimatedRobotPose> rejectedPoses = new ArrayList<>(poseEstimators.size());
        List<Pose3d> usedAprilTags = new ArrayList<>();
        List<Pose3d> rejectedAprilTags = new ArrayList<>();

        for (int i = 0; i < cameras.size(); i++) {
            PhotonCamera photonCamera = cameras.get(i);
            PhotonPoseEstimator poseEstimator = poseEstimators.get(i);
            PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
            List<Pose3d> usedAprilTagsForCamera = new ArrayList<>();
            List<Pose3d> rejectedAprilTagsForCamera = new ArrayList<>();

            if (pipelineResult.getMultiTagResult().estimatedPose.isPresent) {
                Transform3d fieldToCamera = pipelineResult.getMultiTagResult().estimatedPose.best;
                //TODO: compare this with the EstimatedRobotPose result from below
            }

            if (pipelineResult.hasTargets()) {
                int numAprilTagsSeen = pipelineResult.getTargets().size();

                for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
                    double poseAmbiguity = target.getPoseAmbiguity();
                    boolean isUnambiguous = poseAmbiguity < Constants.Vision.POSE_AMBIGUITY_CUTOFF && poseAmbiguity >= 0;
                    boolean isCloseEnough = target.getBestCameraToTarget().getTranslation().getNorm() < Constants.Vision.DISTANCE_CUTOFF;
                    boolean shouldUseTarget = (isUnambiguous || numAprilTagsSeen > 1) && isCloseEnough;
                    Optional<Pose3d> targetPose = fieldLayout.getTagPose(target.getFiducialId());
                    if (targetPose.isPresent()) {
                        Pose3d tagPose = targetPose.get();

                        if (shouldUseTarget) {
                            usedAprilTagsForCamera.add(tagPose);
                        } else {
                            rejectedAprilTagsForCamera.add(tagPose);
                        }
                    }
                }

                if (!usedAprilTagsForCamera.isEmpty()) {
                    usedAprilTags.addAll(usedAprilTagsForCamera);
                }
                if (!rejectedAprilTagsForCamera.isEmpty()) {
                    rejectedAprilTags.addAll(rejectedAprilTagsForCamera);
                }
            }

            Optional<EstimatedRobotPose> poseEstimateResult = poseEstimator.update(pipelineResult);
            if (poseEstimateResult.isPresent()) {
                EstimatedRobotPose poseEstimate = poseEstimateResult.get();
                Pose3d estimatedPose = poseEstimate.estimatedPose;
                double poseTimestamp = poseEstimate.timestampSeconds;

                if (!usedAprilTagsForCamera.isEmpty()) {
                    // Do not use pose if robot pose is off the field 
                    // TODO: comparze Z values if we care about them in 2025 game
                    if (estimatedPose.getX() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                        || estimatedPose.getX() > Constants.Game.FIELD_LENGTH_METERS + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                        || estimatedPose.getY() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                        || estimatedPose.getY() > Constants.Game.FIELD_WIDTH_METERS + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS) {
                            rejectedPoses.add(poseEstimate);
                            continue;
                    }

                    acceptedPoses.add(poseEstimate);
                } else {
                    rejectedPoses.add(poseEstimate);
                }
            }
        }

        if (!usedAprilTags.isEmpty()) {
            acceptedTagPublisher.accept(usedAprilTags.toArray(new Pose3d[0]));
        }
        if (!rejectedAprilTags.isEmpty()) {
            rejectedTagPublisher.accept(rejectedAprilTags.toArray(new Pose3d[0]));
        }

        if (!acceptedPoses.isEmpty()) {
            acceptedVisionPosePublisher.accept(acceptedPoses.toArray(new Pose3d[0]));
        }
        if (!rejectedPoses.isEmpty()) {
            rejectedVisionPosePublisher.accept(rejectedPoses.toArray(new Pose3d[0]));
        }

        return acceptedPoses;
    }

    /*
     * 
    public Matrix<N3, N1> getVisionMeasurementStandardDeviation(EstimatedRobotPose estimation) {
        //TODO: run some unit tests with sample numbers to verify this logic is correct / produces desirable results
        double smallestTargetDistance = Double.POSITIVE_INFINITY;  //in meters
        boolean singleTarget = estimation.targetsUsed.size() == 1;
        double poseAmbiguityFactor = 1;  //default is for the case of multiple targets

        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            //x = forward, y = left, z = up - from the camera
            Transform3d transform3d = target.getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
            if (distance < smallestTargetDistance) {
                smallestTargetDistance = distance;
            }
        }

        if (singleTarget) {
            double ambiguityScore = estimation.targetsUsed.get(0).getPoseAmbiguity() + Constants.Vision.POSE_AMBIGUITY_SHIFTER;
            poseAmbiguityFactor = Math.max(1, ambiguityScore * Constants.Vision.POSE_AMBIGUITY_MULTIPLIER);
        }

        double targetDistanceAdjusted = Math.max(0, smallestTargetDistance - Constants.Vision.NOISY_DISTANCE_METERS);
        double weightedDistance = Math.max(1, targetDistanceAdjusted * Constants.Vision.DISTANCE_WEIGHT); //weighted distance (further distances will be trusted less than shorter)
        double targetScore = weightedDistance * poseAmbiguityFactor;
        double weightedTagPresence = (1 + ((estimation.targetsUsed.size() - 1) * Constants.Vision.TAG_PRESENCE_WEIGHT));  //more targets = more confidence
        double confidenceMultiplier = Math.max(1, targetScore / weightedTagPresence);

        //Note: smaller numbers = trust vision measurements more, larger numbers = trust vision measurements less
        return Constants.Vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }
    */

    public void takeRawImageSnapshot() {
        for (PhotonCamera camera : cameras) {
            camera.takeInputSnapshot();
        }
    }

    public void takeImageSnapshot() {
        for (PhotonCamera camera : cameras) {
            camera.takeOutputSnapshot();
        }
    }
}