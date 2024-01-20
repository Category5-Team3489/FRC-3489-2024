package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.*;

public class Drive extends CommandBase {
    // State
    private final RobotContainer robotContainer;
    private final Drivetrain drivetrain;
    private SlewRateLimiter fullSpeedXRateLimiter = new SlewRateLimiter(FullSpeedRateLimiter100PercentPerSecond);
    private SlewRateLimiter fullSpeedYRateLimiter = new SlewRateLimiter(FullSpeedRateLimiter100PercentPerSecond);

    public Drive(RobotContainer robotContainer, Drivetrain drivetrain) {
        this.robotContainer = robotContainer;
        this.drivetrain = drivetrain;
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!DriverStation.isTeleopEnabled()) {
            fullSpeedXRateLimiter.reset(0);
            fullSpeedYRateLimiter.reset(0);

            drivetrain.driveFieldRelative(0, 0, 0, 0, false);
            return;
        }

        double speedLimiterPercent = robotContainer.input.getDriveSpeedLimiterPercent();

        double xPercent = robotContainer.input.getDriveXPercent();
        if (speedLimiterPercent == 1.0) {
            xPercent = fullSpeedXRateLimiter.calculate(xPercent);
        }
        else {
            fullSpeedXRateLimiter.reset(0);
        }
        double xMetersPerSecond = xPercent * TheoreticalMaxVelocityMetersPerSecond;

        double yPercent = robotContainer.input.getDriveYPercent();
        if (speedLimiterPercent == 1.0) {
            yPercent = fullSpeedYRateLimiter.calculate(yPercent);
        }
        else {
            fullSpeedYRateLimiter.reset(0);
        }
        double yMetersPerSecond = yPercent * TheoreticalMaxVelocityMetersPerSecond;

        if (xMetersPerSecond == 0 && yMetersPerSecond == 0) {
            int degrees = robotContainer.input.getDrivePovAngleDegrees();
            if (degrees != -1) {
                degrees += 90;

                xPercent = Math.sin(Math.toRadians(degrees));
                yPercent = Math.cos(Math.toRadians(degrees));

                xMetersPerSecond = xPercent * PovSpeedMetersPerSecond;
                yMetersPerSecond = yPercent * PovSpeedMetersPerSecond;
            }
        }

        double omegaPercent = robotContainer.input.getDriveOmegaPercent();
        double omegaRadiansPerSecond = omegaPercent * TheoreticalMaxAngularVelocityRadiansPerSecond;

        if (omegaRadiansPerSecond == 0) {
            drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond, speedLimiterPercent, null, null);
        }
        else {
            drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, speedLimiterPercent, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        fullSpeedXRateLimiter.reset(0);
        fullSpeedYRateLimiter.reset(0);
    }
}
