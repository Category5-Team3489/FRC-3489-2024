package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Cat5Utils;
import frc.robot.Constants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

public class SourceCenterLine3Piece extends Command {

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
    Command autoShoot2 = new AutoShoot();

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]

    double speedMultiplier = 0.5; // [0, 1]

    // Rotate/drive to side
    double percentY = Cat5Utils.Blue(-0.5) * MaxMetersPerSecond * speedMultiplier;
    double percentX = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega = Cat5Utils.Blue(-0.5) * MaxMetersPerSecond * speedMultiplier;

    // intake drive
    double percentY2 = Cat5Utils.Blue(0.010) * MaxMetersPerSecond * speedMultiplier;
    double percentX2 = 0.6 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega2 = 0 * MaxMetersPerSecond * speedMultiplier;

    // drive back
    double percentY3 = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentX3 = -0.3 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega3 = Cat5Utils.Blue(0.1) * MaxMetersPerSecond * speedMultiplier;

    // drive side
    double percentY4 = 0.5 * MaxMetersPerSecond * speedMultiplier;
    double percentX4 = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega4 = Cat5Utils.Blue(-0.1) * MaxMetersPerSecond * speedMultiplier;

    double driveTimeSeconds = 0.36;
    double driveTimeSeconds2 = 1.52;
    double driveTimeSeconds3 = 1;
    double driveTimeSeconds4 = 0.36;

    Command closeShootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    AutonomousDrive driveCommandSideRotate = new AutonomousDrive(percentX, percentY, percentOmega);

    AutonomousDrive driveCommandIntake = new AutonomousDrive(percentX2, percentY2, percentOmega2);

    AutonomousDrive driveCommandBack = new AutonomousDrive(percentX3, percentY3, percentOmega3);

    AutonomousDrive driveSide = new AutonomousDrive(percentX4, percentY4, percentOmega4);

    public Command SourceCenterLine3Piece() {

        // return autoShoot
        return Commands.parallel(closeShootCommand, Commands.waitSeconds(3))
                .andThen(Commands.parallel(index.indexCommand(IndexState.Intake)),
                        Commands.waitSeconds(0.5))

                // Drive to side
                .andThen(driveCommandSideRotate.withTimeout(driveTimeSeconds))

                // Start Intake
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))

                // Drive forward to
                .andThen(driveCommandIntake.withTimeout(driveTimeSeconds2))

                // Stop when laser
                .andThen(() -> laserTrigger
                        // .debounce(0.29, DebounceType.kRising)
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                            }
                        })))

                // Drive Back to see april tag
                .andThen(driveCommandBack.withTimeout(driveTimeSeconds3))

                // Auto shoot
                .andThen(() -> autoShoot2.schedule())
                .andThen(Commands.print("PART ONE DONE"))

                .withName("Source 3 piece part one");
    }

}
