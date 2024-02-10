package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class ManualShoot extends Command {

    private final ShooterSpeed shooterSpeed;
    private final ShooterAngle shooterAngle;
    private final Index belt;
    private double angle;
    private final double speed;

    public ManualShoot(double speed, ShooterSpeed shooterSpeed, ShooterAngle shooterAngle, double angle, Index belt) {
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
        shooterSpeed.setSpeedRps(speed);
        // move belt index
        // belt.moveIndexShooter(IndexState.BeltShooter);
    }
}

// manual shooting
// adjust angle
// adjust speed
// move belt index