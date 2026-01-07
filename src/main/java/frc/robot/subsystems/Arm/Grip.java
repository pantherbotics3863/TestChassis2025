// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Arfan
package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {
  /** Creates a new Grip. */

  private final TalonFX motor = new TalonFX(0);
  public Grip() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public Command runCurrent(){
    return null;
  }

  public Command getCurrent(){
    return null;
  }
}
