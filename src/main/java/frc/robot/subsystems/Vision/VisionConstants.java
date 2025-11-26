// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class VisionConstants {
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // Super helpful above ^
    
    // The Robot to the Camera Transform
    public static final Transform3d kRobotToCamera = new Transform3d(
        Inches.of(6.5), Inches.of(3.5), Inches.of(20.5), 
        new Rotation3d(Degrees.of(80),Degrees.of(0),Degrees.of(0))
    );

    public static final Transform3d kCameraToRobot = kRobotToCamera.inverse();
    
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    public static final Distance kDesiredTargetRange = Inches.of(12);
    
}