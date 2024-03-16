package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class SetShooterSpeedAngleDifferent extends SequentialCommandGroup {
    // This command is done when:
    // Shooter is at correct angle and speed
    // Then belt ran for a certain amount of time
    // Then a certain delay passed
    // The shooter angle, shooter speed, and belt are set to defaults

    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    public SetShooterSpeedAngleDifferent(double angleDegrees, double speedPercent, double topSpeedPercent) {
        Command angleCommand = shooterAngle.updateCommand(angleDegrees);
        Command speedCommand = shooterSpeed.updateCommand(topSpeedPercent, speedPercent);

        addCommands(
            Commands.runOnce(() -> {
                System.out.println("SetShooterSpeedAndAngle Command scheduled");
                angleCommand.schedule();
                speedCommand.schedule();
            }),
            Commands.waitSeconds(0.25));
    }
}