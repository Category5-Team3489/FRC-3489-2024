package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeUntilDetectionAngle;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class ShootIntakeAutoShoot extends Command {

    private final Drivetrain drivetrain = Drivetrain.get();
    private final Index index = Index.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command closeShootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command autoShoot = new AutoShoot().withTimeout(2);

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]
    double percentY = 0;
    double percentX = 0.3;
    double percentOmega = 0;
    double driveTimeSeconds = 2.5; // 3 was to far for limelight-- 2 was not enough for intake

    double speedMultiplier = 0.5; // [0, 1]

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandBack = drivetrain.applyRequest(() -> drive
            .withVelocityX(-percentX * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final Intake intake = Intake.get();
    final ShooterAngle shooterAngle = ShooterAngle.get();

    final IntakeUntilDetectionAngle intakeUntilDetection = new IntakeUntilDetectionAngle();

    // Command shootCommand = Commands.parallel(closeShootCommand,
    // Commands.waitSeconds(3))
    // .andThen(Commands.parallel(shooterIndex), Commands.waitSeconds(2));

    public Command shootIntakeAutoShoot() {
        // shooter angle & speed 3 secs
        return Commands.parallel(closeShootCommand, Commands.waitSeconds(3))

                // TODO find out why the parallel is being weird but working

                // shoot
                .andThen(Commands.parallel(shooterIndex), Commands.waitSeconds(2))
                .andThen(() -> closeShootCommand.cancel())

                // TODO Test with only starting intake once

                // Set Intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle())
                        .schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))
                // drives forward with timeout
                .andThen(driveCommandForward.withTimeout(driveTimeSeconds))

                // If laser sensor detects note for more than 0.29 seconds, stop intake/index
                .andThen(() -> laserTrigger
                        .debounce(0.29, DebounceType.kRising)
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                            }
                        })))
                // drives back with timeout
                .andThen(driveCommandBack.withTimeout(1))
                // auto shoots
                .andThen(() -> autoShoot.schedule())

                .withName("ShootIntakeAutoShoot");
    }

}
