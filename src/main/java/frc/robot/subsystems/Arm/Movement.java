// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Rohan
package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Movement extends SubsystemBase {
  private final double gearRatio = 0;
  private final TalonFXS leftMotor = new TalonFXS(16, "rio");
  private final TalonFXS rightMotor = new TalonFXS(19, "rio");
  private final TalonFXSConfiguration defaultConfig = new TalonFXSConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withCommutation(
        new CommutationConfigs()
          .withMotorArrangement(
            MotorArrangementValue.Minion_JST
          )
      )
      .withSlot0(
        new Slot0Configs()
        .withKP(0.1)
        .withKI(0)
        .withKD(0)
        .withKV(0.12)
      )
      ;

  private double maxRotation = 0;
  /** Creates a new Movement. */
  public Movement() {

    leftMotor.getConfigurator().apply(defaultConfig);

    rightMotor.getConfigurator().apply(defaultConfig.withMotorOutput(
      new MotorOutputConfigs()
    .withInverted(InvertedValue.CounterClockwise_Positive)
    ));


    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
      

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setRotation(Angle angle) {
    return null;
  }

  public Angle getRotation() {
    return null;
  }

  public Command runRotationalSpeed(AngularVelocity speed) {
    return Commands.run(()->{
      rightMotor.setControl(new VelocityVoltage(speed));
      leftMotor.setControl(new VelocityVoltage(speed));
    }, this)
    .finallyDo(()->{
      rightMotor.stopMotor();
      leftMotor.stopMotor();
    });
  }

  public Command runFlexSpeed(AngularVelocity speed) {
    return Commands.run(()->{
      rightMotor.setControl(new VelocityVoltage(speed));
      leftMotor.setControl(new VelocityVoltage(speed));
    }, this)
    .finallyDo(()->{
      rightMotor.stopMotor();
      leftMotor.stopMotor();
    });
  }

  public Command doSomethingAnythingImBeggingYou(){
    return Commands.run(()->{
      rightMotor.setVoltage(10);
    }, this)
    .finallyDo(()->{
      rightMotor.stopMotor();
    })
    ;
  }

}
