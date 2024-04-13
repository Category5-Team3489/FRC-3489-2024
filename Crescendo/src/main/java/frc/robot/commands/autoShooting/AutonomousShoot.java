package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSpeed;
import frc.robot.subsystems.ShooterAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Drivetrain;

public class AutonomousShoot extends SequentialCommandGroup {
    private final AutoDrive autoDrive = new AutoDrive();
    private final AutoAim autoAim = new AutoAim(autoDrive);
    private final Command spinUpShooter = ShooterSpeed.get().updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent);
    private final Command angleCommand = ShooterAngle.get()
            .updateCommand(() -> AutoShoot
                    .getShooterAngle(AutoShoot.estimateFloorDistance(AprilLimelight.get().getTargetY())));
    private final Command indexCommand = Index.get().indexCommand(IndexState.Intake);

    // https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html#getinterruptionbehavior

    public AutonomousShoot() {
        addCommands(
            Commands.print("Autonomous shoot started"),
            Commands.race(
                autoDrive,
                autoAim,
                spinUpShooter,
                angleCommand,
                Commands.print("Drive, Aim, Shooter"),
                Commands.sequence(
                    Commands.print("Waiting for shooter to spin up"),
                    Commands.waitSeconds(1.2),
                    Commands.print("Wait for alignment"),
                    Commands.waitUntil(() -> autoAim.isAligned()),
                    Commands.print("Auto aim aligned"),
                    indexCommand,
                    Commands.print("Index command start"),
                    Commands.waitSeconds(2),
                    Commands.print("Index command finished")
                )
            ),
            Commands.print("Autonomous shoot finished")
        );

        // addCommands(
        //         Commands.print("Start AutonomousShoot"),
        //         Commands.runOnce(() -> {
        //             autoDrive.schedule();
        //             autoAim.schedule();
        //             System.out.println("Drive");
        //             spinUpShooter.schedule();
        //             angleCommand.schedule();
        //             System.out.println("Shooter ");

        //         }),

        //         Commands.print("End AutonomousShoot scheduling"),
        //         Commands.waitSeconds(1.2),
        //         Commands.waitUntil(() -> autoAim.isAligned()),
        //         Commands.runOnce(() -> indexCommand.schedule()),
        //         Commands.waitSeconds(2),
        //         Commands.runOnce(() -> {
        //             autoDrive.cancel();
        //             autoAim.cancel();

        //             spinUpShooter.cancel();
        //             angleCommand.cancel();

        //             indexCommand.cancel();
        //         }));

        // addRequirements(Drivetrain.get(), ShooterAngle.get(), ShooterSpeed.get(), Index.get());
    }
}