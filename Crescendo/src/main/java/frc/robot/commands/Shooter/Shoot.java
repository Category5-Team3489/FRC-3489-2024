package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Enums.ShooterState;
import frc.robot.commands.Belt.ManualSetBelt;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class Shoot extends SequentialCommandGroup {

        public Shoot(DoubleSupplier getAngleDegrees, ShooterState shooterState, Shooter shooter,
                        ShooterAngle shooterAngle,
                        Belt belt, double angle) {

                addCommands(
                                Commands.parallel(
                                                new SetShooterAngle(shooterAngle, angle),
                                                new SetShooterSpeed(shooter, shooterState),
                                                Commands.sequence(
                                                                Commands.waitSeconds(0.25),
                                                                Commands.parallel(
                                                                                Commands.waitUntil(() -> shooter
                                                                                                .isAtTargetSpeed()),
                                                                                Commands.waitUntil(() -> shooterAngle
                                                                                                .isAtTarget())),
                                                                new ManualSetBelt(belt, 0.5),
                                                                Commands.waitSeconds(1.0))));
        }

        // This command is done when:
        // Shooter is at correct angle and speed
        // Then belt ran for a certain amount of time
        // Then a certain delay passed
        // The shooter angle, shooter speed, and belt are set to defaults
}
