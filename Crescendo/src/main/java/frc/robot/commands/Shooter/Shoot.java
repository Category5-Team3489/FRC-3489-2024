package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.belt.ManualSetBelt;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class Shoot extends SequentialCommandGroup {
    public Shoot(
            DoubleSupplier angleDegreesSupplier, DoubleSupplier speedRpsSupplier,
            ShooterSpeed shooter, ShooterAngle shooterAngle, Belt belt) {
        addCommands(
                Commands.parallel(
                        new SetShooterAngle(shooterAngle, angleDegreesSupplier),
                        new SetShooterSpeed(speedRpsSupplier, shooter),
                        Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.parallel(
                                        Commands.waitUntil(() -> shooter.isAtTarget()),
                                        Commands.waitUntil(() -> shooterAngle.isAtTarget())),
                                new ManualSetBelt(belt, 0.5).withTimeout(0.7),
                                Commands.waitSeconds(1.0))));
    }

    // This command is done when:
    // Shooter is at correct angle and speed
    // Then belt ran for a certain amount of time
    // Then a certain delay passed
    // The shooter angle, shooter speed, and belt are set to defaults
}
