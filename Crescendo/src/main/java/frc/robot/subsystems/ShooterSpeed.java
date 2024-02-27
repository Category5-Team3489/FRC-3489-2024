package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSpeed extends SubsystemBase {
    // region ---------------- Singleton ----------------
    private static ShooterSpeed instance = new ShooterSpeed();

    public static ShooterSpeed get() {
        return instance;
    }
    // endregion

    // region ---------------- Constants ----------------
    public static final double ShooterIntakeSpeedPercent = -0.4;
    private static final double AllowedErrorRotationsPerSecond = 5;

    private class MotorBuilder {
        private static final int TopCanId = 13;
        private static final int BottomCanId = 12;

        private static final double VoltsPerRotationPerSecondOfError = 12.0 / 50; // kP
        private static final double VoltsPerRotationOfError = 0.0; // kI
        private static final double VoltsPerRotationPerSecondSquaredOfError = 0.0; // kD
        // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V,
        // 1 / 8.33 = 0.12 volts / rotation per second
        private static final double VoltsPerRotationPerSecond = 0.12; // kV

        private TalonFX build(int id) {
            TalonFX motor = new TalonFX(id);

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

            return motor;
        }
    }
    // endregion

    // region ---------------- Devices ----------------
    private final TalonFX topMotor = new MotorBuilder().build(13);
    private final TalonFX bottomMotor = new MotorBuilder().build(12);
    // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
    private final VelocityVoltage velocity = new VelocityVoltage(0);
    // endregion

    private ShooterSpeed() {
        bottomMotor.setInverted(true);
    }

    // region Subsystem
    //TODO add bottom motor
    public void setSpeedRps(double speedRps) {
        if (speedRps == 0) {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        } else {
            velocity.Slot = 0; // Closed loop slot index 0
            topMotor.setControl(velocity.withVelocity(speedRps));
            bottomMotor.setControl(velocity.withVelocity(speedRps));

            //TODO Remove after testing
            // topMotor.set(speedRps);
            // bottomMotor.set(speedRps);

        }
    }

    public void setSpeedPercent(double speedPercent) {
        if (speedPercent == 0) {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        } else {
            topMotor.set(speedPercent);
            bottomMotor.set(speedPercent);
        }
    }

    /**
     * Returns true if the magnitude of the error is less than
     * AllowedErrorRotationsPerSecond
     */
    public boolean isAtTargetSpeed() {
        double errorRps = topMotor.getClosedLoopError().getValueAsDouble();
        return Math.abs(errorRps) <= AllowedErrorRotationsPerSecond;
    }
    // endregion

    // region Commands
    public Command updateCommand(DoubleSupplier speedRpsSupplier) {
        return Commands.run(() -> {
            setSpeedRps(speedRpsSupplier.getAsDouble());
        }, this);
    }

    public Command setMotorPercent(DoubleSupplier speedRpsSupplier) {
        return Commands.runOnce(() -> {
            setSpeedPercent(speedRpsSupplier.getAsDouble());
        }, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        }, this);
    }
    // endregion
}
