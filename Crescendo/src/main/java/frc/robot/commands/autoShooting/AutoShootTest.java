package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAngle;

public class AutoShootTest extends Command{

    private Drivetrain drivetrain;
    private ShooterAngle shooterAngle;
    private AprilLimelight limelight;

    public AutoShootTest(Drivetrain drivetrain, ShooterAngle shooterAngle, AprilLimelight limelight) {
        this.drivetrain = drivetrain;
        this.shooterAngle = shooterAngle;
        this.limelight = limelight;

        addRequirements(drivetrain, shooterAngle, limelight);
    }

    @Override
    public void execute() {
        double targetX = limelight.getTargetX();
    }

    //rotate to target based on tx/pid
    //set shooter angle based on distance from apriltag
    //set shooter speed to constant speed
    //index piece
    
}
