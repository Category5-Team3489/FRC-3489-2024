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

public class Part2OfCenterLine extends Command {

    private Index index = Index.get();
    private ShooterAngle shooterAngle = ShooterAngle.get();
    private Intake intake = Intake.get();

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

    // drive side
    double percentY = 0.5 * MaxMetersPerSecond * speedMultiplier;
    double percentX = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega = Cat5Utils.Blue(-0.1) * MaxMetersPerSecond * speedMultiplier;

    // drive forward to intake
    double percentY2 = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentX2 = 0.5 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega2 = Cat5Utils.Blue(0) * MaxMetersPerSecond * speedMultiplier;

    // drive back to see april tag
    double percentY3 = 0 * MaxMetersPerSecond * speedMultiplier;
    double percentX3 = -0.5 * MaxMetersPerSecond * speedMultiplier;
    double percentOmega3 = Cat5Utils.Blue(0) * MaxMetersPerSecond * speedMultiplier;

    double driveTimeSeconds = 0.36;
    double driveTimeSeconds2 = 5;
    double driveTimeSeconds3 = 5;

    Command driveSide = new AutonomousDrive(percentX, percentY, percentOmega).withTimeout(driveTimeSeconds);
    Command driveForward = new AutonomousDrive(percentX2, percentY2, percentOmega2).withTimeout(driveTimeSeconds2);
    Command driveBack = new AutonomousDrive(percentX3, percentY3, percentOmega3).withTimeout(driveTimeSeconds3);

    public Command SourceCenterLine3Piece() {
        return driveSide
                .andThen(Commands.print("PART 2 START"))
                .andThen(() -> shooterAngle.updateCommand(ShooterAngleState.Max.getAngle()).schedule())
                .andThen(intake.intakeCommand(IntakeState.centerIn, IntakeState.falconIn))
                .andThen(index.indexCommand(IndexState.Intake))
                .andThen(driveForward)
                // Stop when laser
                .andThen(() -> laserTrigger
                        .onTrue(Commands.runOnce(() -> {
                            if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                            }
                        })))
                .andThen(driveBack)
                .andThen(Commands.print("PART 2 DONE"))

                .andThen(autoShoot2);

    }

}
