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
                        Commands.runOnce(() -> {
                            try {
                            autoDrive.schedule();
                            autoAim.schedule();
                            System.out.println("Drive/Aim Schedule");
                            spinUpShooter.schedule();
                            angleCommand.schedule();
                            System.out.println("Shoot speed/angle Schedule");

                            } catch (Exception ex) {
                                System.out.println("Scheduling Error: " + ex.getMessage());
                            }

                        }),
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