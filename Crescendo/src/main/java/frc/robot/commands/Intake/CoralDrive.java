package frc.robot.commands.Intake;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CoralDrive extends Command {
    private final Drivetrain drivetrain = Drivetrain.get();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    private double velocityX = 0;
    private double velocityY = 0;
    private double omega = 0;

    public CoralDrive() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(robotCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(omega));
    }

    public void setVelocityX(double velocityX) {
        this.velocityX = velocityX;
    }

    public void setVelocityY(double velocityY) {
        this.velocityY = velocityY;
    }

    public void setOmega(double omega) {
        this.omega = omega;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Coral Drive End Method");
    }
}