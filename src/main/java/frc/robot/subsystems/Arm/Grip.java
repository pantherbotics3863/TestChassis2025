package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {
    private final TalonFX motor = new TalonFX(0);

    private static final double OPEN_VELOCITY = 0.4;
    private static final double CLOSE_VELOCITY = -0.3;
    private static final double CURRENT_LIMIT = 25.0; 
    private static final double WAIT_TIME = 0.25; // Seconds to ignore inrush

    public Grip() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }

    public Command openClaw() {
        Timer timer = new Timer();
        return this.run(() -> motor.set(OPEN_VELOCITY))
            .beforeStarting(timer::restart)
            // Only stop if the timer has passed the "inrush" period
            .until(() -> timer.get() > WAIT_TIME && motor.getStatorCurrent().getValueAsDouble() > CURRENT_LIMIT)
            .finallyDo(() -> motor.set(0));
    }

    public Command closeClaw() {
      // Motor runs while button is held
      return this.runEnd(
          () -> motor.set(CLOSE_VELOCITY),
          () -> motor.set(0)
      );
    }

}