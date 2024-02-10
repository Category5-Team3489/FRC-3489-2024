package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSpeed extends SubsystemBase {
    private static ShooterSpeed instance = new ShooterSpeed();

    public static ShooterSpeed get() {
        return instance;
    }

    private static final double ShooterIntakeSpeedPercent = -0.4;

    // TODO SET CAN ID
    private static final int MotorId = 14;

    // PID
    private static final double VoltsPerRotationPerSecondOfError = 12.0 / 50; // kP
    private static final double VoltsPerRotationOfError = 0.0; // kI
    private static final double VoltsPerRotationPerSecondSquaredOfError = 0.0; // kD

    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1 / 8.33 = 0.12
    // volts / rotation per second
    private static final double VoltsPerRotationPerSecond = 0.12; // kV

    // Shooter is at its target speed if the error is within
    // plus or minus this value
    private static final double AllowedErrorRotationsPerSecond = 5;

    private final TalonFX motor = new TalonFX(0);

    // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
    private final VelocityVoltage velocity = new VelocityVoltage(0);

    private ShooterSpeed() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = VoltsPerRotationPerSecondOfError;
        configs.Slot0.kI = VoltsPerRotationOfError;
        configs.Slot0.kD = VoltsPerRotationPerSecondSquaredOfError;
        configs.Slot0.kV = VoltsPerRotationPerSecond;

        // Try 5 times to apply the configurations and print an error if it always fails
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    // Rev up motors
    public void setSpeedRps(double speedRps) {
        velocity.Slot = 0; // Closed loop slot index 0
        motor.setControl(velocity.withVelocity(speedRps));
    }

    public void setShooterIntakeSpeed() {
        motor.set(ShooterIntakeSpeedPercent);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isAtTargetSpeed() {
        double errorRps = motor.getClosedLoopError().getValueAsDouble();
        return Math.abs(errorRps) <= AllowedErrorRotationsPerSecond;
    }

    public Command shootNote() {
        return Commands.run(() -> {

        });
    }

    public Command shooterStop() {
        return Commands.run(() -> {

        });
    }
}
