package frc.robot.commands.autoShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AprilLimelight;

public class AutoAim extends Command {
    private final AprilLimelight aprilLimelight = AprilLimelight.get();
    private final AutoDrive autoDrive;

    private final double targetXRange = 5;
    private final double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;

    public AutoAim(AutoDrive autoDrive) {
        this.autoDrive = autoDrive;

        addRequirements();
    }

    @Override
    public void execute() {
        double targetX = aprilLimelight.getTargetX();
        double targetV = aprilLimelight.getTargetVisible();

        // Return if april tag is not visible
        if (targetV == 0) {
            autoDrive.setOmega(0);
            autoDrive.setVelocityX(-0.5);
            autoDrive.setVelocityY(0);
            return;
        }

        // Apriltag is visible
        if (isAligned()) {
            autoDrive.setOmega(0);
        } else if (targetX < 0) {
            autoDrive.setOmega(-rotationSpeed);
        } else if (targetX > 0) {
            autoDrive.setOmega(rotationSpeed);
        }
    }

    public boolean isAligned() {
        double targetX = aprilLimelight.getTargetX();
        return Math.abs(targetX) < targetXRange;
    }
}