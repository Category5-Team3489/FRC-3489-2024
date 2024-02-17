package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.Shoot;
import frc.robot.enums.LimelightPipeline;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class AutoShoot extends Command {
    private final AprilLimelight limelight = AprilLimelight.get();
    private final Drivetrain drivetrain = Drivetrain.get();
    private final Index index = Index.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    public AutoShoot() {

    }

    @Override
    public void initialize() {
        if (index.isNoteDetected()) {
            this.cancel();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

// rotate robot
// adjust angle
// adjust speed
// move belt to shoot



// - press button
// - check if note is loaded (maybe)
// - detect april tag
// - calculate offset from center of shooter
// - calculate disatance from shooter
// - start spinning shooter motors
// - set shooter angle
// - use drive train to turn shooter
// - validate shooter is within tolerances
// - index note
// - stop shooter motors
// - reset shooter angle for next intake