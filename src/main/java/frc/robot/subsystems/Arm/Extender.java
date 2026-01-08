// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Yilin
package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Inches;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  private final double maxRotations = 1;
  private final Distance maxLength = Inches.of(12);
  private final TalonFX motor = new TalonFX(15);

  /** Creates a new Extender. */
  public Extender() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Distance getPosition() {
    return null;
  }

  public Command setPosition(Distance distance) {
    return null;
  }

  public Command runSpeed(Velocity velocity) {
    return null;
  }
}
