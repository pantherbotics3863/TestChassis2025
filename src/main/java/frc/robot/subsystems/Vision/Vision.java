// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final PhotonCamera cam;
  private final PhotonPoseEstimator photonPoseEstimator;

  public PhotonCamera getCamera(){
    return cam;
  }

  public PhotonPipelineResult getLatestResult(){
    return cam.getLatestResult();
  }

  public Optional<PhotonTrackedTarget> getBestTargetPose(){
    PhotonPipelineResult result = cam.getLatestResult();
    if (result.hasTargets()){
      return Optional.of(result.getBestTarget());
    }
    return Optional.empty();
  }

  public Optional<Pose3d> getFieldPose() {
    // Optional<PhotonTrackedTarget> potentialTarget = getTargetPose();
    // if (potentialTarget.isPresent()){
    //   PhotonTrackedTarget target = potentialTarget.get();
    //   if (VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()){
    //     return Optional.of( PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kCameraToRobot) );
    //   }
    // }
    // return Optional.empty();

    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    for (PhotonPipelineResult newResult : cam.getAllUnreadResults()){
      estimatedPose = photonPoseEstimator.update(newResult);
    }
    if (estimatedPose.isPresent()){
      return Optional.of(estimatedPose.get().estimatedPose);
    } else {
      return Optional.empty();
    }
    
  }

  public Vision(String cameraNameString) {
    cam = new PhotonCamera(cameraNameString);
    photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCamera);
  }
}
