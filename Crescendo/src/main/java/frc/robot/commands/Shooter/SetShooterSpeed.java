package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Enums.ShooterState;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends Command {
    
    private final Shooter shooter;
    private final ShooterState shooterState;

    public SetShooterSpeed(Shooter shooter, ShooterState shooterState) {
        this.shooter = shooter;
        this.shooterState = shooterState;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // adjust speed
        shooter.setShooterState(shooterState);
    }
}
