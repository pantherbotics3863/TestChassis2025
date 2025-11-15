// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class VisionConstants {
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // Super helpful above ^
    public static final Transform3d cameraToRobot = new Transform3d(Inches.of(5.5), Inches.of(-2), Inches.of(16), 
        new Rotation3d(Degrees.of(0),Degrees.of(10),Degrees.of(-25))
    );
}
