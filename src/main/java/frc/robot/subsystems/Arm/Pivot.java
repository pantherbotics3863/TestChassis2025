package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

  private static final double GEAR_RATIO = 3.0;

  private final TalonFX motor = new TalonFX(14, "rio");

  private double zeroRotations;

  private final TalonFXConfiguration defaultConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake)
          )
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKV(0.12)
          );

  public Pivot() {
    motor.getConfigurator().apply(defaultConfig);
    zeroRotations = motor.getPosition().getValueAsDouble();
  }

  public Angle getAngle() {
    double motorRotations =
        motor.getPosition().getValueAsDouble() - zeroRotations;

    double armRotations = motorRotations / GEAR_RATIO;
    return Units.Rotations.of(armRotations);
  }

  public Command setAngle() {
    return Commands.runOnce(() -> {
      zeroRotations = motor.getPosition().getValueAsDouble();
    }, this);
  }

  public Command runSpeed(AngularVelocity speed) {
    return Commands.run(
        () -> motor.setControl(new VelocityVoltage(speed)),
        this
    ).finallyDo(motor::stopMotor);
  }
}
