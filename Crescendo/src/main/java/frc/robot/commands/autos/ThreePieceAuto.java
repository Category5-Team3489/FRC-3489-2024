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

public class ThreePieceAuto {
    private final Drivetrain drivetrain = Drivetrain.get();
    private final Index index = Index.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final Intake intake = Intake.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command autoShoot = new AutoShoot();
    Command autoShoot2 = new AutoShoot().withTimeout(2);
    Command autoShoot3 = new AutoShoot().withTimeout(2);

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]

    // Rotate/drive side
    double percentY = 0;
    double percentX = 0;
    double percentOmega = -2;

    // intake drive
    double percentY2 = 0;
    double percentX2 = 0.3;
    double percentOmega2 = 0;

    // drive back
    double percentY3 = 0;
    double percentX3 = -0.3;
    double percentOmega3 = 0;

    // rotate
    double percentY4 = 0;
    double percentX4 = 0;
    double percentOmega4 = -1;

    // drive/intake
    double percentY5 = 0;
    double percentX5 = 0.3;
    double percentOmega5 = 0;

    // drive back
    double percentY6 = 0;
    double percentX6 = -0.3;
    double percentOmega6 = 0;

    double driveTimeSeconds = 0.2; // 3 was to far for limelight-- 2 was not enough for intake
    double driveTimeSeconds2 = 2.5;
    double driveTimeSeconds3 = 1;
    double driveTimeSeconds4 = 1;
    double driveTimeSeconds5 = 4;
    double driveTimeSeconds6 = 4;

    double speedMultiplier = 0.5; // [0, 1]

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

    Command driveCommandRotate2 = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX3 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY3 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega3 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandIntake2 = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX3 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY3 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega3 * MaxRadiansPerSecond * speedMultiplier));

    Command driveCommandBack2 = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX3 * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY3 * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega3 * MaxRadiansPerSecond * speedMultiplier));

    public Command threePieceAuto() {
                                        //Auto Shoot
        return Commands.runOnce(() -> autoShoot.schedule())
                        //rotate to straight
                .andThen(driveCommandSideRotate.withTimeout(driveTimeSeconds))

                //start intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                //drive forward
                .andThen(driveCommandIntake.withTimeout(driveTimeSeconds2))

                //stop intake when piece detected
                .andThen(() -> laserTrigger.debounce(0.29, DebounceType.kRising).onTrue(Commands.runOnce(() -> {
                    if (intake.hasIntakeBeenSet) {
                        intake.stop();
                        index.stop();
                    }
                })))

                //drive back to see april tag
                .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

                //auto shoot
                .andThen(autoShoot2)

                //Rotate to straight
                .andThen(driveCommandRotate2.withTimeout(driveTimeSeconds4))

                //Start intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                //drive forward to centerline piece
                .andThen(driveCommandIntake2.withTimeout(driveTimeSeconds5))

                //stop intake when piece detected
                .andThen(() -> laserTrigger.debounce(0.29, DebounceType.kRising).onTrue(Commands.runOnce(() -> {
                    if (intake.hasIntakeBeenSet) {
                        intake.stop();
                        index.stop();
                    }
                })))

                //drive back to see apriltag
                .andThen(driveCommandBack2.withTimeout(driveTimeSeconds6))

                //auto shoot
                .andThen(autoShoot3)

                .withName("ThreePiece");

    }
}

// side shoot auto
// start facing april tag
// auto stoot
// rotate to straight
// drive forward and intake
// drive back to see april tag
// auto shoot
// rotate
// intake/drive forward
// drive back
// auto shoot
