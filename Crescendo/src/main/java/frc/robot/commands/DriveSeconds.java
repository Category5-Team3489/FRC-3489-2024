package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveSeconds extends Command{
    private final Drivetrain drivetrain;

    private Timer timer = new Timer();
    private double seconds;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public DriveSeconds(Drivetrain drivetrain, double seconds, double speed) {
        this.drivetrain = drivetrain;
        this.seconds = seconds;

        drivetrain.applyRequest(() -> drive
                .withVelocityX((16.5 / 3.281) * speed) // (MaxMetersPerSecond) * 0.15 //0.75434 meters
                .withVelocityY((16.5 / 3.281) * speed) // (MaxMetersPerSecond) * 0.15
                .withRotationalRate(
                        ((16.5 / 3.281) / Math.hypot(DrivetrainConstants.kFrontLeftXPosInches / 39.37,
                                DrivetrainConstants.kFrontLeftYPosInches / 39.37)) * 0) // (MaxMetersPerSecond)
        );
        System.out.println("DRIVE SECONDS");
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
  }

    

}
