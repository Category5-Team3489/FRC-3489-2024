package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;

public class Shoot extends Command {
    // TODO Stop Index/Intake After shoot
    Index index = Index.get();

    Command shootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    public Command shoot() {

        return Commands.parallel(shootCommand, Commands.waitSeconds(2))
                .andThen(() -> shooterIndex.schedule()).withTimeout(2)
                .withName("Shoot");
    }

}
