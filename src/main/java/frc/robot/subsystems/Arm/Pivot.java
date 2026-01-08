// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Arfan
package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final double gearRatio = 3;
  private final TalonFX motor = new TalonFX(14);
  /** Creates a new Pivot. */
  public Pivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Angle getAngle() {
    return null;
  }

  public Command setAngle(Angle angle){
    return null;
  }

  public Command runSpeed(Velocity speed){
    return null;
  }


}
