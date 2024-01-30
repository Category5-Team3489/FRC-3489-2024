package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;

public class SetShooterAngle extends Command {

    private final ShooterAngle shooterAngle;
    private double angle;

    public SetShooterAngle(ShooterAngle shooterAngle, double angle) {
        this.shooterAngle = shooterAngle;
        this.angle = angle;

        addRequirements(shooterAngle);
    }

    @Override
    public void initialize() {
        // adjust angle
        shooterAngle.adjustAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}