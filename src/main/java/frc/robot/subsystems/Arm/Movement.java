// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Movement extends SubsystemBase {
  private final double gearRatio = 0;
  private final TalonFX leftMotor = new TalonFX(0);
  private final TalonFX rightMotor = new TalonFX(0);
  private double maxRotation = 0;
  /** Creates a new Movement. */
  public Movement() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setRotation() {
    return null;
  }

  public Angle getRotation() {
    return null;
  }

  public Command runRotationalSpeed() {
    return null;
  }

  public Command runFlexSpeed() {
    return null;
  }

}
