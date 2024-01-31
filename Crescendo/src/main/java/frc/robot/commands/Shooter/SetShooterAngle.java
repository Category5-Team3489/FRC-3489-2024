package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;

public class SetShooterAngle extends Command {

    private final ShooterAngle shooterAngle;
    private DoubleSupplier angleDegreesSupplier;

    public SetShooterAngle(ShooterAngle shooterAngle, DoubleSupplier angleDegreesSupplier) {
        this.shooterAngle = shooterAngle;
        this.angleDegreesSupplier = angleDegreesSupplier;

        addRequirements(shooterAngle);
    }

    @Override
    public void initialize() {
        // adjust angle
        shooterAngle.setAngle(angleDegreesSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}