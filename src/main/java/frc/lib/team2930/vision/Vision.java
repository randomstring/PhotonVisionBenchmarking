package frc.lib.team2930.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team2930.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.TunableNumber;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  ArrayList<CameraResultProcessingPackage> allCameraResultProcessingPackages =
      new ArrayList<CameraResultProcessingPackage>();

  DoubleSupplier robotPitch;
  DoubleSupplier robotRoll;

  AprilTagFieldLayout aprilTagLayout;

  boolean useVisionForPoseEstimation = true;
  boolean useMaxDistanceAwayFromExistingEstimate = true;
  boolean useMaxPitchRoll = true;

  private static double MAX_ALLOWABLE_PITCH = 3;
  private static double MAX_ALLOWABLE_ROLL = 3;

  // private static double MAX_TAG_LOG_TIME = 0.1;

  private static TunableNumber thetaStdDevCoefficient =
      new TunableNumber("vision/thetaStdDevCoefficient", 0.075);
  private static TunableNumber xyStdDevCoefficient =
      new TunableNumber("vision/xyStdDevCoefficient", 0.075);

  private HashMap<Integer, Double> lastTagDetectionTimes = new HashMap<Integer, Double>();

  private List<Pose3d> actualPosesUsedInPoseEstimator = new ArrayList<>();

  public Vision(DoubleSupplier pitch, DoubleSupplier roll, AprilTagFieldLayout layout, VisionIOConfig... VisionIOConfigs) {
    this.robotPitch = pitch;
    this.robotRoll = roll;

    aprilTagLayout = layout;

    for (VisionIOConfig config : VisionIOConfigs) {
      allCameraResultProcessingPackages.add(
        new CameraResultProcessingPackage(config, aprilTagLayout));
      config.visionIO.setVision(this);
    }

    // log all robotToCamera constants, useful for cameraOverride view mode in advantage scope
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      Logger.getInstance()
          .recordOutput(
              "Vision/" + cameraPackage.name + "CameraConstant",
              new Pose3d().transformBy(cameraPackage.RobotToCamera));
    }

    for (AprilTag tag : aprilTagLayout.getTags()) {
      Logger.getInstance().recordOutput("Vision/AllAprilTags3D/" + tag.ID, tag.pose);
    }

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));
  }


  public static double calculateStandardDeviation(double Kp, double distanceFromTag, int numTagsSeen) {
    // minimum std deviation is 1 cm or .01 rad (0.6 degrees)
    return Math.min(0.01, (Kp * Math.pow(distanceFromTag + 1.0, 2.0)) / ((double) Math.pow(numTagsSeen, 2.0)));
  }


  public void processCameraVisionUpdateByName(String name) {
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      if (name == cameraPackage.name) {
        processCameraVisionUpdate(cameraPackage);
      }
    }
  }

  public void processCameraVisionUpdate(CameraResultProcessingPackage cameraPackage) {
      cameraPackage.visionIO.updateInputs(cameraPackage.visionIOInputs);
      Logger.getInstance()
          .processInputs("Vision/" + cameraPackage.name, cameraPackage.visionIOInputs);

    boolean processVision = true;
    if (!useVisionForPoseEstimation) {
      processVision = false;
      cameraPackage.loggedFields = VisionProcessingLoggedFields.unsuccessfulStatus(VisionProcessingStatus.NOT_PROCESSING_VISION);
    }

    if (Math.abs(robotPitch.getAsDouble()) >= MAX_ALLOWABLE_PITCH
        || Math.abs(robotRoll.getAsDouble()) >= MAX_ALLOWABLE_ROLL) {
    
      processVision = false;
      cameraPackage.loggedFields = VisionProcessingLoggedFields.unsuccessfulStatus(VisionProcessingStatus.GYRO_ANGLE_NOT_VALID);
    }

    if (processVision) {
        var fieldsToLog = processVision(cameraPackage);
        cameraPackage.loggedFields = fieldsToLog;
    }

    logVisionProcessingStatusAndFields(cameraPackage);

    // log the poses used in the latest updates
 

    Logger.getInstance().recordOutput("Vision/useVision", useVisionForPoseEstimation);
    Logger.getInstance()
        .recordOutput(
            "Vision/useMaxDistanceAwayFromExistingEstimate",
            useMaxDistanceAwayFromExistingEstimate);
    Logger.getInstance().recordOutput("Vision/useMaxPitchRoll", useMaxPitchRoll);

    // FIXME: for now log global position estimate here 
    Logger.getInstance().recordOutput("Vision/robotPose", RobotOdometry.getEstimatedPosition());

  }

  @Override
  public void periodic() {
 
    List<Pose3d> allAtThisVeryMomentVisibleTags = new ArrayList<>();

    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() <= 0.02) {
        // note tags visible in the last robot clock cycle
        var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
        allAtThisVeryMomentVisibleTags.add(tagPose.get());
      }
    }

    Logger.getInstance()
        .recordOutput(
            "Vision/currentVisibleTags_EXACT_MOMENT",
            allAtThisVeryMomentVisibleTags.toArray(
                new Pose3d[allAtThisVeryMomentVisibleTags.size()]));

    Logger.getInstance()
        .recordOutput(
            "Vision/actual_poses_used_in_pose_estimator",
            actualPosesUsedInPoseEstimator.toArray(
                new Pose3d[actualPosesUsedInPoseEstimator.size()]));

    // clear poses after we log them
    actualPosesUsedInPoseEstimator.clear();

    Logger.getInstance().recordOutput("Vision/useVision", useVisionForPoseEstimation);
    Logger.getInstance()
        .recordOutput(
            "Vision/useMaxDistanceAwayFromExistingEstimate",
            useMaxDistanceAwayFromExistingEstimate);
    Logger.getInstance().recordOutput("Vision/useMaxPitchRoll", useMaxPitchRoll);

    // FIXME: for now log global position estimate here 
    Logger.getInstance().recordOutput("Vision/robotPose", RobotOdometry.getEstimatedPosition());

    // isConnected
    // clean logging
  }

  public VisionProcessingLoggedFields processVision(CameraResultProcessingPackage cameraPackage) {
    PhotonPipelineResult cameraResult;
    double currentResultTimeStamp;

    //SwerveDrivePoseEstimator globalPoseEstimator = RobotOdometry.getInstance().getPoseEstimator();
    Pose2d prevEstimatedRobotPose = RobotOdometry.getEstimatedPosition();

    Pose3d newCalculatedRobotPose;

    double xyStandardDeviation;
    double thetaStandardDeviation;

    // storing fields to log
    double tagAmbiguity;
    double distanceFromTag;
    Pose3d cameraPose;

    synchronized (cameraPackage.visionIOInputs) {
      cameraResult = cameraPackage.visionIOInputs.lastResult;
      currentResultTimeStamp = cameraPackage.visionIOInputs.lastTimestamp;
    }

    if (cameraPackage.lastProcessedResultTimeStamp >= currentResultTimeStamp) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.NOT_A_NEW_RESULT);
    }

    cameraPackage.lastProcessedResultTimeStamp = currentResultTimeStamp;

    // String ROOT_LOG_PATH = "Vision/" + cameraPackage.name + "/";

    ArrayList<PhotonTrackedTarget> cleanTargets = new ArrayList<PhotonTrackedTarget>();

    for (PhotonTrackedTarget target : cameraResult.getTargets()) {
      if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
        cleanTargets.add(target);
      }
    }

    cleanTargets.forEach(
        (PhotonTrackedTarget tag) ->
            lastTagDetectionTimes.put(tag.getFiducialId(), Timer.getFPGATimestamp()));

    var numTargetsSeen = cleanTargets.size();

    if (numTargetsSeen == 0) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.NO_TARGETS_VISIBLE);
    }

    // --------------------- EXPERIMENTAL

    if (numTargetsSeen == 1) {
      PhotonTrackedTarget singularTag = cameraResult.getTargets().get(0);

      if (!isValidTarget(singularTag)) {
        return (singularTag.getPoseAmbiguity() >= VisionConstants.MAXIMUM_AMBIGUITY)
            ? VisionProcessingLoggedFields.unsuccessfulStatus(
                VisionProcessingStatus.INVALID_TAG_AMBIGUITY_TOO_HIGH)
            : VisionProcessingLoggedFields.unsuccessfulStatus(VisionProcessingStatus.INVALID_TAG);
      }
    }

    Optional<EstimatedRobotPose> photonPoseEstimatorOptionalResult;

    cameraPackage.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    photonPoseEstimatorOptionalResult = cameraPackage.photonPoseEstimator.update(cameraResult);

    if (photonPoseEstimatorOptionalResult.isEmpty()) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY);
    }

    newCalculatedRobotPose = photonPoseEstimatorOptionalResult.get().estimatedPose;

    // logged fields
    tagAmbiguity = 0.0;
    cameraPose = newCalculatedRobotPose.transformBy(cameraPackage.RobotToCamera);

    double totalDistance = 0.0;
    for (PhotonTrackedTarget tag : cleanTargets) {
      totalDistance +=
          aprilTagLayout
              .getTagPose(tag.getFiducialId())
              .get()
              .getTranslation()
              .getDistance(newCalculatedRobotPose.getTranslation());
    }

    distanceFromTag = totalDistance / (double) cleanTargets.size();

 
      // FIX ME: removed for simulation
      // if (cameraResult.getTargets().size() > 1) {
      //   return VisionProcessingLoggedFields.unsuccessfulStatus(
      //       VisionProcessingStatus.LOGIC_ERROR_EXPECTED_1_TARGET);
      // }

    var distanceFromExistingPoseEstimate =
        prevEstimatedRobotPose
            .getTranslation()
            .getDistance(
                new Translation2d(newCalculatedRobotPose.getX(), newCalculatedRobotPose.getY()));

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate
            > (VisionConstants.MAX_VALID_DISTANCE_AWAY_METERS * numTargetsSeen))) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.TOO_FAR_FROM_EXISTING_ESTIMATE);
    }

    xyStandardDeviation = calculateStandardDeviation(xyStdDevCoefficient.get(), distanceFromTag, numTargetsSeen);
    thetaStandardDeviation = calculateStandardDeviation(thetaStdDevCoefficient.get(), distanceFromTag, numTargetsSeen);


     RobotOdometry.addVisionMeasurement(
          newCalculatedRobotPose.toPose2d(),
          currentResultTimeStamp,
          VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation));
 

    actualPosesUsedInPoseEstimator.add(newCalculatedRobotPose);

    return new VisionProcessingLoggedFields(
        VisionProcessingStatus.SUCCESSFUL,
        numTargetsSeen,
        tagAmbiguity,
        distanceFromTag,
        distanceFromExistingPoseEstimate,
        xyStandardDeviation,
        thetaStandardDeviation,
        currentResultTimeStamp,
        cameraPose,
        newCalculatedRobotPose);

    // log num targets seen

  }

  public void setAllCameraPackageUnsuccessfulStatus(VisionProcessingStatus status) {
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.loggedFields = VisionProcessingLoggedFields.unsuccessfulStatus(status);
    }
  }

  // public void setAllCameraPackageFieldsToLog(VisionProcessingLoggedFields fieldsToLog) {
  //   for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
  //     cameraPackage.fieldsToLog = fieldsToLog;
  //   }
  // }

  public void logVisionProcessingStatusAndFields(CameraResultProcessingPackage cameraPackage) {

    String ROOT_TABLE_PATH = "Vision/" + cameraPackage.name + "/";

    var fieldsToLog = cameraPackage.loggedFields;

    Logger logger = Logger.getInstance();

    logger.recordOutput(
        ROOT_TABLE_PATH + "*STATUS",
        fieldsToLog.status().name() + ": " + fieldsToLog.status().logOutput);

    logger.recordOutput(ROOT_TABLE_PATH + "calculated_robotPose_3d", fieldsToLog.robotPose3d());
    logger.recordOutput(
        ROOT_TABLE_PATH + "calculated_robotPose_2d", fieldsToLog.robotPose3d().toPose2d());
    logger.recordOutput(ROOT_TABLE_PATH + "camera_pose_3d", fieldsToLog.cameraPose());
    logger.recordOutput(ROOT_TABLE_PATH + "num_seen_targets", fieldsToLog.numSeenTargets());
    logger.recordOutput(ROOT_TABLE_PATH + "processed_timestamp", fieldsToLog.processedTimeStamp());
    logger.recordOutput(
        ROOT_TABLE_PATH + "distance_from_existing_pose_estimate",
        fieldsToLog.distanceFromExistingPoseEstimate());
    logger.recordOutput(ROOT_TABLE_PATH + "distance_from_tag", fieldsToLog.distanceFromTag());
    logger.recordOutput(ROOT_TABLE_PATH + "tag_ambiguity", fieldsToLog.tagAmbiguity());
    logger.recordOutput(
        ROOT_TABLE_PATH + "xy_standard_deviation", fieldsToLog.xyStandardDeviation());
    logger.recordOutput(
        ROOT_TABLE_PATH + "theta_standard_deviation", fieldsToLog.thetaStandardDeviation());

    boolean addedVisionEstimateToPoseEstimator =
        (fieldsToLog.status().equals(VisionProcessingStatus.SUCCESSFUL)) ? true : false;

    logger.recordOutput(
        ROOT_TABLE_PATH + "added_vision_measurement_to_pose_estimator(AKA SUCCESSFUL?)",
        addedVisionEstimateToPoseEstimator);
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && aprilTagLayout.getTagPose(target.getFiducialId()).isPresent();
  }

  public void useMaxDistanceAwayFromExistingEstimate(boolean value) {
    useMaxDistanceAwayFromExistingEstimate = value;
  }

  private class CameraResultProcessingPackage {
    final VisionIO visionIO;
    final VisionIOInputs visionIOInputs;
    final PhotonPoseEstimator photonPoseEstimator;
    final Transform3d RobotToCamera;
    final String name;

    public double lastProcessedResultTimeStamp;
    public VisionProcessingLoggedFields loggedFields;

    public CameraResultProcessingPackage(
        VisionIOConfig config, AprilTagFieldLayout aprilTagFieldLayout) {
      this.visionIO = config.visionIO;
      this.visionIOInputs = new VisionIOInputs();

      this.RobotToCamera = config.robotToCamera;

      this.name = config.name;

      this.photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, visionIO.getCamera(), RobotToCamera);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

      lastProcessedResultTimeStamp = 0.0;
    }

  }

  private record VisionProcessingLoggedFields(
      VisionProcessingStatus status,
      double numSeenTargets,
      double tagAmbiguity,
      double distanceFromTag,
      double distanceFromExistingPoseEstimate,
      double xyStandardDeviation,
      double thetaStandardDeviation,
      double processedTimeStamp,
      Pose3d cameraPose,
      Pose3d robotPose3d) {

    private VisionProcessingLoggedFields {}

    private VisionProcessingLoggedFields(VisionProcessingStatus status) {
      this(status, -1, -1, -1, -1, -1, -1, -1, new Pose3d(), new Pose3d());
    }

    public static VisionProcessingLoggedFields DEFAULT_LOG_VALUES =
        new VisionProcessingLoggedFields(
            // probably make the poses like -10, -10, 0 to move them off screen and make it obvious
            // its bad results
            VisionProcessingStatus.UNKNOWN, -1, -1, -1, -1, -1, -1, -1, new Pose3d(), new Pose3d());

    public static VisionProcessingLoggedFields unsuccessfulStatus(VisionProcessingStatus status) {
      return new VisionProcessingLoggedFields(status);
    }
  }

  private enum VisionProcessingStatus {
    NOT_PROCESSING_VISION(""),
    GYRO_ANGLE_NOT_VALID(""),
    NOT_A_NEW_RESULT(""),
    PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY(""),
    NO_TARGETS_VISIBLE(""),

    LOGIC_ERROR_EXPECTED_1_TARGET(""),

    TOO_FAR_FROM_EXISTING_ESTIMATE(""),

    INVALID_TAG_AMBIGUITY_TOO_HIGH(""),
    INVALID_TAG(""),

    TAG_NOT_IN_LAYOUT(""),

    SUCCESSFUL(""),
    UNKNOWN("UNKNOWN");

    public final String logOutput;

    private VisionProcessingStatus(String logOutput) {
      this.logOutput = logOutput;
    }
  }
}
