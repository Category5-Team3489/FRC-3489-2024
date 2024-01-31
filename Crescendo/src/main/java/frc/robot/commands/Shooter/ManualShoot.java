package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Enums.BeltState;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class ManualShoot extends Command {

    private final ShooterSpeed shooterSpeed;
    private final ShooterAngle shooterAngle;
    private final Belt belt;
    private double angle;
    private final double speed;

    public ManualShoot(double speed, ShooterSpeed shooterSpeed, ShooterAngle shooterAngle, double angle, Belt belt) {
        this.shooterSpeed = shooterSpeed;
        this.shooterAngle = shooterAngle;
        this.belt = belt;
        this.angle = angle;
        this.speed = speed;

        addRequirements(shooterSpeed, shooterAngle);
    }

    @Override
    public void execute() {
        // adjust angle
        shooterAngle.setAngle(angle);
        // adjust speed
        shooterSpeed.setSpeed(speed);
        // move belt index
        belt.moveBeltShooter(BeltState.BeltShooter);
    }
}

// manual shooting
// adjust angle
// adjust speed
// move belt index