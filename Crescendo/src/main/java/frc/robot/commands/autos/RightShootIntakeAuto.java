package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class RightShootIntakeAuto extends Command{

    private Index index = Index.get();
    private ShooterAngle shooterAngle = ShooterAngle.get();
    private Intake intake = Intake.get();
    private Drivetrain drivetrain = Drivetrain.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    Trigger laserTrigger = new Trigger(index.laserSensor::get);

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command autoShoot = new AutoShoot().withTimeout(2);
    Command autoShoot2 = new AutoShoot().withTimeout(2);

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]

    // Rotate/drive side
    double percentY = 0;
    double percentX = 0;
    double percentOmega = 0.2;

    // intake drive
    double percentY2 = 0;
    double percentX2 = 0.3;
    double percentOmega2 = 0;

    // drive back
    double percentY3 = 0;
    double percentX3 = -0.3;
    double percentOmega3 = -0.1;

    double driveTimeSeconds = 0.6;
    double driveTimeSeconds2 = 2.5;
    double driveTimeSeconds3 = 1;

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

    // start facing april tag
    // auto stoot
    // rotate to straight
    // drive forward and intake
    // drive back to see april tag
    // auto shoot

    // TODO if that does not work
    // return autoShoot

    public Command rightShootIntakeAuto() {

        // return autoShoot
        return Commands.parallel(closeShootCommand, Commands.waitSeconds(3))

        
                .andThen(driveCommandSideRotate.withTimeout(driveTimeSeconds))

                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                .andThen(driveCommandIntake.withTimeout(driveTimeSeconds2))

                .andThen(() -> laserTrigger.debounce(0.29, DebounceType.kRising).onTrue(Commands.runOnce(() -> {
                    if (intake.hasIntakeBeenSet) {
                        intake.stop();
                        index.stop();
                    }
                })))

                .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

                .andThen(autoShoot2)

                .withName("RightSideShootIntakeAuto");
    }

}
