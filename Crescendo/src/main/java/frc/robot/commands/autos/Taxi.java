package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.subsystems.Drivetrain;

public class Taxi {

    private final Drivetrain drivetrain = Drivetrain.get();

    private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
    private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

    // final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxMetersPerSecond * 0.1)
    //         .withRotationalDeadband(MaxRadiansPerSecond * 0.1)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Ensure percentages are greater than the 0.1 percent deadband above
    // Domain is [-1, 1]
    double percentY = 0.3;
    double percentX = 0;
    double percentOmega = 0;
    double driveTimeSeconds = 3;

    AutonomousDrive drive = new AutonomousDrive(0.3, 0, 0);

    double speedMultiplier = 0.5; // [0, 1]

    // Command command = drivetrain.applyRequest(() -> drive
    //         .withVelocityX(percentY * MaxMetersPerSecond * speedMultiplier)
    //         .withVelocityY(-percentX * MaxMetersPerSecond * speedMultiplier)
    //         .withRotationalRate(-percentOmega * MaxRadiansPerSecond * speedMultiplier));

    public Command taxi() {
        // return command.withTimeout(driveTimeSeconds).withName("Taxi");
        return drive.withTimeout(5)
        .withName("Taxi");
    }

}
