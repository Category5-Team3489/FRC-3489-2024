package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSpeed;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Index;

public class AutonomousShoot extends SequentialCommandGroup {
    private final AutoDrive autoDrive = new AutoDrive();
    private final AutoAim autoAim = new AutoAim(autoDrive);
    private final Command spinUpShooter = ShooterSpeed.get().updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent);
    private final Command angleCommand = ShooterAngle.get()
            .updateCommand(() -> AutoShoot
                    .getShooterAngle(AutoShoot.estimateFloorDistance(AprilLimelight.get().getTargetY())));
    private final Command indexCommand = Index.get().indexCommand(IndexState.Intake);

    public AutonomousShoot() {
        addCommands(
                autoDrive.withTimeout(2),
                autoAim.withTimeout(1),
                Commands.print("Drive/Aim Schedule"),
                spinUpShooter.withTimeout(1),
                Commands.print("Shoot speed/angle Schedule"),
                angleCommand.withTimeout(1),
                Commands.print("Run Once Done"),
                Commands.waitSeconds(1.2),
                Commands.waitUntil(() -> autoAim.isAligned()),
                Commands.runOnce(() -> indexCommand.schedule()),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> {
                    autoDrive.cancel();
                    autoAim.cancel();

                    spinUpShooter.cancel();
                    angleCommand.cancel();

                    indexCommand.cancel();
                }));
    }
}