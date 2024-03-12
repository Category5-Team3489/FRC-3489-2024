package frc.robot.commands.autoShooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAngle;

public class AutoShootTest extends Command {

    private Drivetrain drivetrain = Drivetrain.get();

    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final AprilLimelight limelight = AprilLimelight.get();

    private double xRange = 30;
    private double yRange = 1;

    public PIDController aimController = new PIDController(0.0001, 0, 0);

    public AutoShootTest() {

        addRequirements(drivetrain, shooterAngle, limelight);
    }

    @Override
    public void execute() {
        double X = limelight.getTargetX();
        double Y = limelight.getTargetY();

    }

    // rotate to target based on tx/pid
    // set shooter angle based on distance from apriltag
    // set shooter speed to constant speed
    // index piece

}
