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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {

    private final TalonFXS motor = new TalonFXS(18, "rio");

    private static final AngularVelocity OPEN_SPEED = Units.RotationsPerSecond.of(1.0);

    private static final AngularVelocity CLOSE_SPEED = Units.RotationsPerSecond.of(-0.7);

    private final TalonFXSConfiguration defaultConfig = new TalonFXSConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withCommutation(
                new CommutationConfigs()
                    .withMotorArrangement(MotorArrangementValue.Minion_JST)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.1)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
            );

    public Grip() {
        motor.getConfigurator().apply(defaultConfig);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command openClaw() {
        return Commands.run(
            () -> motor.setControl(new VelocityVoltage(OPEN_SPEED)),
            this
        ).finallyDo(motor::stopMotor);
    }

    public Command closeClaw() {
        return Commands.run(
            () -> motor.setControl(new VelocityVoltage(CLOSE_SPEED)),
            this
        ).finallyDo(motor::stopMotor);
    }
}



/*
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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {

    private final TalonFXS motor = new TalonFXS(18, "rio");

    private static final AngularVelocity OPEN_SPEED =
        Units.RotationsPerSecond.of(0.4);

    private static final AngularVelocity CLOSE_SPEED =
        Units.RotationsPerSecond.of(-0.3);

    private static final double CURRENT_LIMIT = 25.0;
    private static final double INRUSH_IGNORE_TIME = 0.4;

    private final Timer closeTimer = new Timer();

    private final TalonFXSConfiguration defaultConfig =
        new TalonFXSConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withCommutation(
                new CommutationConfigs()
                    .withMotorArrangement(MotorArrangementValue.Minion_JST)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.1)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
            );

    public Grip() {
        motor.getConfigurator().apply(defaultConfig);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command openClaw() {
        return Commands.run(
            () -> motor.setControl(new VelocityVoltage(OPEN_SPEED)),
            this
        ).finallyDo(motor::stopMotor);
    }

    public Command closeClaw() {
        return Commands.run(() -> {
            motor.setControl(new VelocityVoltage(CLOSE_SPEED));

            double current = motor.getStatorCurrent().getValueAsDouble();

            if (closeTimer.hasElapsed(INRUSH_IGNORE_TIME) && current > CURRENT_LIMIT) {
                motor.stopMotor();
            }
        }, this)
        .beforeStarting(() -> {
            closeTimer.reset();
            closeTimer.start();
        })
        .finallyDo(() -> {
            closeTimer.stop();
            motor.stopMotor();
        });
    }
}


 */