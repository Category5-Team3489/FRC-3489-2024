package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSpeed extends SubsystemBase {
    private static ShooterSpeed instance = new ShooterSpeed();

    public static ShooterSpeed get() {
        return instance;
    }

    private static final int TopMotorCanId = 13;
    private static final int BottomMotorCanId = 12;

    private final VoltageOut voltageOut = new VoltageOut(0);

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

    private void setTopSpeedPercent(double speedPercent) {
        if (speedPercent == 0) {
            topMotor.stopMotor();
            speedPercent = 0;

            //New code for setting idle instead of stoping the motor
            // double voltage = 0.20 * 12;
            // voltageOut.Output = voltage;
            // topMotor.setControl(voltageOut);
        } else {
            double voltage = speedPercent * 12;
            voltageOut.Output = voltage;
            topMotor.setControl(voltageOut);
            // bottomMotor.setControl(voltageOut);

            // topMotor.set(speedPercent);
            // bottomMotor.set(speedPercent);

            if (Math.abs(speedPercent) > 1) {
                System.out.println("CHECK THE CODE: The speed percent is too high: " + speedPercent);
            }
        }

        this.speedPercent = speedPercent;
    }

    private void setBottomSpeedPercent(double speedPercent) {
        if (speedPercent == 0) {
            // This is what we had when it set it to 0 instead on idle 20
            bottomMotor.stopMotor();
            speedPercent = 0;


            // double voltage = 0.20 * 12;
            // voltageOut.Output = voltage;
            // bottomMotor.setControl(voltageOut);
        } else {
            double voltage = speedPercent * 12;
            voltageOut.Output = voltage;
            // topMotor.setControl(voltageOut);
            bottomMotor.setControl(voltageOut);

            // topMotor.set(speedPercent);
            // bottomMotor.set(speedPercent);

            if (Math.abs(speedPercent) > 1) {
                System.out.println("CHECK THE CODE: The speed percent is too high: " + speedPercent);
            }
        }

        this.speedPercent = speedPercent;
    }

    public Command updateCommand(double speedPercent) {
        // System.out.println("==================================UPDATE Command");
        return updateCommand(() -> speedPercent);
    }

    public Command updateCommand(DoubleSupplier speedPercentSupplier) {
        return Commands.runEnd(() -> {
            setBottomSpeedPercent(speedPercentSupplier.getAsDouble());
            setTopSpeedPercent(speedPercentSupplier.getAsDouble());

        }, () -> {
            setBottomSpeedPercent(0.20);
            setTopSpeedPercent(0.20);

        }, this);
    }

    public Command updateCommand(double topSpeed, double bottomSpeed) {
        return Commands.runEnd(() -> {
            setTopSpeedPercent(topSpeed);
            setBottomSpeedPercent(bottomSpeed);
        }, () -> {
            setTopSpeedPercent(0);
            setBottomSpeedPercent(0);
        }, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            setTopSpeedPercent(0.2);
            setBottomSpeedPercent(0.2);
        }, this);
    }

    public Command stopButton() {
        return Commands.runOnce(() -> {
            bottomMotor.stopMotor();
            topMotor.stopMotor();
        }, this);
    }
}
