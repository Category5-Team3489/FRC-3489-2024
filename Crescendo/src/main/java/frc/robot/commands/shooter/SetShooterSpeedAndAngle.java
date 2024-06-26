package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class SetShooterSpeedAndAngle extends SequentialCommandGroup {
    // This command is done when:
    // Shooter is at correct angle and speed
    // Then belt ran for a certain amount of time
    // Then a certain delay passed
    // The shooter angle, shooter speed, and belt are set to defaults

    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    public SetShooterSpeedAndAngle(double angleDegrees, double speedPercent) {
        Command angleCommand = shooterAngle.updateCommand(angleDegrees);
        Command speedCommand = shooterSpeed.updateCommand(speedPercent);

        addCommands(
            Commands.runOnce(() -> {
                System.out.println("SetShooterSpeedAndAngle Command scheduled");
                angleCommand.schedule();
                speedCommand.schedule();
            }),
            Commands.waitSeconds(0.25));
    }
}