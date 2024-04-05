package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class RightCenterLine3Piece {
    private Index index = Index.get();
    private ShooterAngle shooterAngle = ShooterAngle.get();
    private Intake intake = Intake.get();
    private Drivetrain drivetrain = Drivetrain.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    private boolean canAutoShoot = true;

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // TODO test with time out (remove if does not work)
    Command autoShoot = new AutoShoot();
    Command autoShoot2 = new AutoShoot().onlyWhile(() -> canAutoShoot);
    Command autoShoot3 = new AutoShoot();

    // make command to schedule auto shoot
    Command autoShootSchedule = Commands.runOnce(() -> autoShoot2.schedule());

    Command autoIntakeCommand = new CoralIntake();

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]

    // Rotate/drive side
    double percentY = 0.3;
    double percentX = 0;
    double percentOmega = 0.2;
    double driveTimeSeconds = 0.6;

    // intake drive
    double percentY2 = 0;
    double percentX2 = 0.3;
    double percentOmega2 = 0;
    double driveTimeSeconds2 = 2.5;

    // drive back
    double percentY3 = 0;
    double percentX3 = -0.3;
    double percentOmega3 = -0.1;
    double driveTimeSeconds3 = 1;

    // rotate to straight, drive to right, forward
    double percentY4 = 0;
    double percentX4 = 0.5;
    double percentOmega4 = 0;
    double driveTimeSeconds4 = 0.4;

    // drive forward
    double percentY5 = 0;
    double percentX5 = 0.5;
    double percentOmega5 = 0;
    double driveTimeSeconds5 = 0.4;

    // drive back to see apriltag
    double percentY6 = 0;
    double percentX6 = 0.5;
    double percentOmega6 = 0;
    double driveTimeSeconds6 = 0.4;

    double speedMultiplier = 0.5; // [0, 1]

    Command closeShootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command driveCommandSideRotate = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandIntake = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX2 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY2 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega2 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandBack = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX3 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY3 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega3 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandRotate = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX4 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY4 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega4 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX5 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY5 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega5 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandBack2 = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX6 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY6 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega6 * MaxRadiansPerSecond * speedMultiplier));

    // Command driveCommandForward = drivetrain.applyRequest(() -> drive
    // .withVelocityX(percentX4 * MaxMetersPerSecond * speedMultiplier)
    // .withVelocityY(-percentY4 * MaxMetersPerSecond * speedMultiplier)
    // .withRotationalRate(-percentOmega4 * MaxRadiansPerSecond * speedMultiplier));

    public Command rightCenterLine3Piece() {
        // manual shoot
        return Commands.parallel(closeShootCommand, Commands.waitSeconds(3))

                // rotate to forward and drive to the side away from the speaker
                .andThen(driveCommandSideRotate.withTimeout(driveTimeSeconds))

                // Start intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                // Drive forward
                .andThen(driveCommandIntake.withTimeout(driveTimeSeconds2))

                // Stop intake when piece detected
                .andThen(() -> laserTrigger.debounce(0.29, DebounceType.kRising)
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                            }
                        })))
                // drive back to see apriltag
                .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

                // Auto shoot
                // .andThen(Commands.race(autoShoot2, Commands.waitSeconds(2)))
                // .andThen(Commands.parallel( autoShootSchedule,
                // Commands.runOnce(() -> Commands.waitSeconds(2))
                // .andThen(Commands.runOnce(() -> canAutoShoot = false))))

                // .andThen(autoShoot2.withTimeout(2).andThen(Commands.runOnce(() ->
                // canAutoShoot = false)))

                // TODO Previous code
                // .andThen(Commands.race(autoShoot2, Commands.waitSeconds(2)))

                // TODO runEnd(parallel(schedule and wait), cancle)
                .andThen(Commands.runEnd(
                        () -> Commands.parallel(Commands.runOnce(() -> autoShoot2.schedule()), Commands.waitSeconds(2)),
                        () -> Commands.runOnce(() -> autoShoot2.cancel())))

                //TODO 

                .andThen(() -> System.out.println("End Auto========++++++++++"))
                // -------- End of two piece

                // Rotate to straight
                .andThen(driveCommandRotate.withTimeout(driveTimeSeconds4))

                // Start intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                // Coral Intake
                .andThen(autoIntakeCommand)

                // Drive forward
                .andThen(driveCommandForward.withTimeout(driveTimeSeconds5))

                // Stop intake when piece detected
                .andThen(() -> laserTrigger.debounce(0.29, DebounceType.kRising)
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                            }
                        })))

                // Drive back to see apriltag
                .andThen(driveCommandBack2.withTimeout(driveTimeSeconds6))

                .andThen(() -> autoShoot3.schedule())

                .withName("RightCenterline3");
    }
}

// Right Two piece
// rotate to straight , drive to right and straight (to blue line?)
// Start intake
// Auto rotate
// Drive forward
// stop intake with laser
// drive back to see apriltag
// auto shoot
