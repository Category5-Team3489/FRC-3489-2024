package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDrive extends Command {

    private final Drivetrain drivetrain = Drivetrain.get();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    private double velocityX = 0;
    private double velocityY = 0;
    private double omega = 0;

    /**
     * Sets the X, Y, and Angular rate of a drive command.
     *
     * <p>
     * <b>
     * Note: Must Include a Timeout.
     * Use {@link Command#withTimeout(double)}
     * </b>
     * </p>
     * 
     * @param velocityX Velocity in the X direction, in m/s
     * @param velocityY Velocity in the Y direction, in m/s
     * @param omega     Angular rate to rotate at, in radians per second
     * 
     */
    public AutonomousDrive(double velocityX, double velocityY, double omega) {
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.omega = omega;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // this.withTimeout(8);
    }

    @Override
    public void execute() {
        drivetrain.setControl(fieldCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(omega));
    }

    // @Override
    // public void end() {
    //     System.out.println("Autonomous Drive Command End");
    // }

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
