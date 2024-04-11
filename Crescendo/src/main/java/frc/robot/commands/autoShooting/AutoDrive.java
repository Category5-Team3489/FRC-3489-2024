package frc.robot.commands.autoShooting;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends Command {
    private final Drivetrain drivetrain = Drivetrain.get();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    private double velocityX = 0;
    private double velocityY = 0;
    private double omega = 0;

    public AutoDrive() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(fieldCentric
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
}