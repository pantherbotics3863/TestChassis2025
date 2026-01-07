package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final double gearRatio = 60;
    private final TalonFX motor = new TalonFX(1);

    private double targetAngle = 0;
    private final double maxSpeed = 0.5;
    private final double kP = 0.02;
    private final double deadzone = 1;

    public Pivot() {
        motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

        this.setDefaultCommand(holdAngleCommand());
    }

    public double getCurrentAngle() {
        return (motor.getPosition().getValueAsDouble() / gearRatio) * 360;
    }

    
    public Command holdAngleCommand() {
        return this.run(() -> {
            double error = targetAngle - getCurrentAngle();

            
            double gravityOffset = 0.05 * Math.cos(Math.toRadians(getCurrentAngle()));

            double speed = error * kP;
            speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

            
            if (Math.abs(error) < deadzone) speed = 0;

            motor.set(ControlModeValue.PercentOutput, speed + gravityOffset);
        });
    }

    
    public Command runManual(double speed) {
        return this.run(() -> {
            motor.set(ControlModeValue.PercentOutput, speed);
            targetAngle = getCurrentAngle(); 
        });
    }


    public Command setAngleCommand(double angle) {
        return this.runOnce(() -> targetAngle = angle);
    }

    
    public Angle getAngle() {
        return Degrees.of(getCurrentAngle());
    }
}
