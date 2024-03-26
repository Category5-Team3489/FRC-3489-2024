package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;

public class ShootTaxi extends Command {

    //The Robot will taxi in the direction the intake is facing when it is powered on

    private final Drivetrain drivetrain = Drivetrain.get();
    private final Index index = Index.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1)
            .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    Command shootCommand = new SetShooterSpeedAndAngle(
            Constants.ShooterAngle.CloseShooterAngle,
            Constants.ShooterSpeed.CloseShooterSpeed).withTimeout(2);

    Command shooterIndex = index.indexCommand(IndexState.Intake);

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]
    double percentY = 0;
    double percentX = 0.3;
    double percentOmega = 0;
    double driveTimeSeconds = 15;

    double speedMultiplier = 0.5; // [0, 1]

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(percentX * MaxMetersPerSecond * speedMultiplier)
            .withVelocityY(-percentY * MaxMetersPerSecond * speedMultiplier)
            .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    public Command shootTaxi() {
        return Commands.parallel(shootCommand, Commands.waitSeconds(3))
                .andThen(Commands.parallel(shooterIndex), Commands.waitSeconds(2)).andThen(driveCommandForward)
                .withTimeout(driveTimeSeconds).withName("ShootTaxi");

    }


}
