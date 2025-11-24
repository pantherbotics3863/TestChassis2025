// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSim extends SubsystemBase {
  /** Creates a new VisionSim. */
  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final SimCameraProperties cameraProperties = new SimCameraProperties();
  private final PhotonCameraSim cammySim;
  

  public VisionSim(PhotonCamera camera) {
    visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

    // Property Setting
    cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(80));
    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(35);
    cameraProperties.setLatencyStdDevMs(5);

    // Actual simulated camera
    cammySim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cammySim, VisionConstants.kRobotToCamera);

  }

  public void visionPeriodic(Pose2d robotPose){
    visionSim.update(robotPose);
  }

  
}
