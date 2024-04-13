package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;

public class ShootTaxi extends SequentialCommandGroup {

    // The Robot will taxi in the direction the intake is facing when it is powered
    // on

    private final Index index = Index.get();

    Command shootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    double speedMultiplier = 0.5; // [0, 1]
    AutonomousDrive drive = new AutonomousDrive(0.3 * speedMultiplier, 0, 0);

    public ShootTaxi() {
        addCommands(
                Commands.parallel( // Set the speed and angle of the shooter for 3 seconds
                        shootCommand,
                        Commands.waitSeconds(3),
                        Commands.print("Shoot command")),
                // Index for 2 seconds to shoot note
                shooterIndex,
                Commands.print("Index"),
                Commands.waitSeconds(2),
                Commands.print("Drive"),
                drive.withTimeout(15));// drive for 15 seconds
    }
}
