package frc.robot.commands;

import frc.robot.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Cardinal extends Command {
    private final Drivetrain drivetrain;
    private double pigeon;

    private double degrees;

    private final double deadbandrangeDegrees = 5;

    public Cardinal(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pigeon = drivetrain.getCurrentHeading();

        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

        drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(((16.5 / 3.281) / Math.hypot(DrivetrainConstants.kFrontLeftXPosInches / 39.37,
                        DrivetrainConstants.kFrontLeftYPosInches / 39.37)) * 0.5) // Drive
        );

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(pigeon - degrees) <= deadbandrangeDegrees;
    }
}
