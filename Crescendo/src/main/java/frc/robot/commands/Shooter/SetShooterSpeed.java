package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSpeed;

public class SetShooterSpeed extends Command {
    private final DoubleSupplier speedRpsSupplier;
    private final ShooterSpeed shooterSpeed;

    public SetShooterSpeed(DoubleSupplier speedRpsSupplier, ShooterSpeed shooterSpeed) {
        this.speedRpsSupplier = speedRpsSupplier;
        this.shooterSpeed = shooterSpeed;

        addRequirements(shooterSpeed);
    }

    @Override
    public void execute() {
        // adjust speed of shooter
        shooterSpeed.setSpeedRps(speedRpsSupplier.getAsDouble());
    }
}
