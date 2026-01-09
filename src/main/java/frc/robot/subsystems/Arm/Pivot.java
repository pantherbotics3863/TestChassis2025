package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Pivot extends SubsystemBase {

  private final double gearRatio = 3.0;
  public final TalonFX motor = new TalonFX(14);

  private double zeroRotations;

  public Pivot() {
    zeroRotations = motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {}

  public Angle getAngle() {
    double motorRotations =
        motor.getPosition().getValueAsDouble() - zeroRotations;

    double armRotations = motorRotations / gearRatio;
    return Units.Rotations.of(armRotations);
  }

  public Command setAngle() {
    return Commands.runOnce(() -> {
      zeroRotations = motor.getPosition().getValueAsDouble();
    }, this);
  }

  public Command runSpeed() {
    return Commands.run(() -> motor.set(0.3), this)
        .finallyDo(() -> motor.set(0));
  }
}
