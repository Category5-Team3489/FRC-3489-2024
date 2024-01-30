package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Enums.BeltState;
import frc.robot.Enums.ShooterState;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class ManualShoot extends Command {

    private final Shooter shooter;
    private final ShooterAngle shooterAngle;
    private final ShooterState shooterState;
    private final Belt belt;
    private double angle;

    public ManualShoot(Shooter shooter, ShooterAngle shooterAngle, double angle, Belt belt, ShooterState shooterState) {
        this.shooter = shooter;
        this.shooterAngle = shooterAngle;
        this.belt = belt;
        this.angle = angle;
        this.shooterState = shooterState;

        addRequirements(shooter, shooterAngle);
    }

    @Override
    public void execute() {
        // adjust angle
        shooterAngle.adjustAngle(angle);
        // adjust speed
        shooter.setShooterState(shooterState);
        // move belt index
        belt.moveBeltShooter(BeltState.BeltShooter);
    }
}

// manual shooting
// adjust angle
// adjust speed
// move belt index