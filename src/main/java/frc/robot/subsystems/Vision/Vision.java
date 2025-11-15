// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;
import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  PhotonCamera cam;

  public PhotonTrackedTarget getTargetPose(){
    return cam.getLatestResult().getBestTarget();
  }

  public Optional<Pose3d> getFieldPose() {
    if (cam.getLatestResult().hasTargets()){
      PhotonTrackedTarget target = getTargetPose();
      if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()){
        return Optional.of( PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.cameraToRobot) );
      }
    }
    return Optional.empty();
  }


  public Vision(String cameraNameString) {
    cam = new PhotonCamera(cameraNameString);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
