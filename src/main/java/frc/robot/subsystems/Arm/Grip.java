package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {

    private final TalonFX motor = new TalonFX(18);

    private static final double OPEN_VELOCITY = 1.0;

    private static final double CLOSE_VELOCITY = -0.7;

   

    public Grip() {
       
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(config);

    }

    public Command openClaw() {
        return this.runEnd(

            () -> motor.set(OPEN_VELOCITY),
            () -> motor.set(0)
        );
    }

    public Command closeClaw() {
        return this.runEnd(

            () -> motor.set(CLOSE_VELOCITY),

            () -> motor.set(0)
        );
    }
}


/*
 package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {

    private final TalonFX motor = new TalonFX(18);

    private static final double OPEN_VELOCITY = 0.4;
    private static final double CLOSE_VELOCITY = -0.3;
    private static final double CURRENT_LIMIT = 25.0;
    private static final double INRUSH_IGNORE_TIME = 0.4;

    private final Timer closeTimer = new Timer();

    public Grip() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }

    public Command openClaw() {
        return this.runEnd(
            () -> motor.set(OPEN_VELOCITY),
            () -> motor.set(0)
        );
    }

    public Command closeClaw() {
        return this.run(() -> {
            motor.set(CLOSE_VELOCITY);

            double current = motor.getStatorCurrent().getValueAsDouble();

            if (closeTimer.hasElapsed(INRUSH_IGNORE_TIME)) {
                if (current > CURRENT_LIMIT) {
                    motor.stopMotor();
                }
            }
        })
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