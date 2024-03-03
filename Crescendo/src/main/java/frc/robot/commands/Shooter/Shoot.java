package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class Shoot extends SequentialCommandGroup {
    // This command is done when:
    // Shooter is at correct angle and speed
    // Then belt ran for a certain amount of time
    // Then a certain delay passed
    // The shooter angle, shooter speed, and belt are set to defaults

    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
    private final Index index = Index.get();

    public Shoot(DoubleSupplier angleDegreesSupplier, DoubleSupplier speedRpsSupplier) {
        System.out.println("Shoot Command scheduled");
        Command angleCommand = shooterAngle.updateCommand(angleDegreesSupplier);
        Command speedCommand = shooterSpeed.updateCommand(speedRpsSupplier);

        addCommands(
                Commands.runOnce(() -> {
                    angleCommand.schedule();
                    speedCommand.schedule();
                    System.out.println("Set Commands Scheduled");
                }),
                Commands.waitSeconds(0.25),
                Commands.waitUntil(() -> shooterAngle.isAtTargetAngle() && shooterSpeed.isAtTargetSpeed()),
                index.indexCommand(IndexState.Intake).withTimeout(0.7),
                Commands.waitSeconds(1),
                Commands.runOnce(() -> {
                    angleCommand.cancel();
                    speedCommand.cancel();
                }));
    }
}