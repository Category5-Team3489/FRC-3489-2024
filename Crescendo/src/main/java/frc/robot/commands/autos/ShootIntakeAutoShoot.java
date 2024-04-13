package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.Intake.AutonomousCoralIntake;
import frc.robot.commands.autoShooting.AutonomousShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class ShootIntakeAutoShoot extends SequentialCommandGroup {

    private final Index index = Index.get();

    Command closeShootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed);

    Command autoShoot = new AutonomousShoot().withTimeout(2);

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    Command driveCommandForward = new AutonomousDrive(0.15, 0, 0).withTimeout(1);
    Command driveCommandBack = new AutonomousDrive(-0.15, 0, 0).withTimeout(2.5);

    Command coralIntake = new AutonomousCoralIntake().withTimeout(5);

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final Intake intake = Intake.get();
    final ShooterAngle shooterAngle = ShooterAngle.get();

    public ShootIntakeAutoShoot() {
        addCommands(
                Commands.parallel(
                        closeShootCommand,
                        Commands.print("Set shooter"),
                        Commands.waitSeconds(3)),

                // shoot
                Commands.print("--------END OF PARALLEL========="),

                shooterIndex,
                Commands.print("Index--"),
                Commands.waitSeconds(2),

                // Drives back off the wall
                Commands.print("Drive Forward"),
                driveCommandForward,

                // Coral Intake: Rotate to game piece, start intake, stop when laser sensor or
                // after 5 seconds
                Commands.print("Coral Intake"),
                coralIntake,

                // drives back with timeout to see april tag
                Commands.print("Drive Back"),
                driveCommandBack,
                Commands.waitSeconds(1),

                // auto shoots
                Commands.print("Auto Shoot"),
                autoShoot);
    }

}
