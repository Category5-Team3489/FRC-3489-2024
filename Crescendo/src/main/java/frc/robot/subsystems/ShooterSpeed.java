package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSpeed extends SubsystemBase {
    private static ShooterSpeed instance = new ShooterSpeed();

    public static ShooterSpeed get() {
        return instance;
    }

    private static final int TopMotorCanId = 13;
    private static final int BottomMotorCanId = 12;

    private final TalonFX topMotor = new TalonFX(TopMotorCanId);
    private final TalonFX bottomMotor = new TalonFX(BottomMotorCanId);

    private double speedPercent = 0;

    private ShooterSpeed() {
        bottomMotor.setInverted(true);

        Shuffleboard.getTab("Main")
                .addDouble("Shooter speed", () -> speedPercent)
                .withSize(1, 1)
                .withPosition(8, 1);
    }

    private void setSpeedPercent(double speedPercent) {
        if (speedPercent == 0) {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
            speedPercent = 0;
        } else {
            topMotor.set(speedPercent);
            bottomMotor.set(speedPercent);

            if (Math.abs(speedPercent) > 1) {
                System.out.println("CHECK THE CODE: The speed percent is too high: " + speedPercent);
            }
        }

        this.speedPercent = speedPercent;
    }

    public Command updateCommand(double speedPercent) {
        return updateCommand(() -> speedPercent);
    }

    public Command updateCommand(DoubleSupplier speedPercentSupplier) {
        return Commands.runOnce(() -> {
            setSpeedPercent(speedPercentSupplier.getAsDouble());
        }, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            setSpeedPercent(0);
        }, this);
    }
}
