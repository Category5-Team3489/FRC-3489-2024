package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class Leave extends SequentialCommandGroup {
    private final Drivetrain drivetrain;

    private Timer timer = new Timer();

    public Leave(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
        timer.start();

        // TODO Adjust Drive Speeds

        drivetrain.applyRequest(() -> drive
                .withVelocityX((16.5 / 3.281) * 0.15) // (MaxMetersPerSecond) * 0.15  //0.75434 meters per second

                // (forward)
                .withVelocityY((16.5 / 3.281) * 0.15) // (MaxMetersPerSecond) * 0.15

                .withRotationalRate(((16.5 / 3.281) / Math.hypot(DrivetrainConstants.kFrontLeftXPosInches / 39.37,
                        DrivetrainConstants.kFrontLeftYPosInches / 39.37)) * 0) // (MaxMetersPerSecond) / ...
                                                                                // counterclockwise?

        );
    }

    
}
