package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.autoShooting.AutoDrive;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class CoralAim extends Command {
    private final AprilLimelight aprilLimelight = AprilLimelight.get();
    private final CoralDrive coralDrive;

    private final double targetXRange = 5;
    private final double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;

    private final double tXRange = 16;

    private Trigger laserTrigger = new Trigger(Index.get().laserSensor::get);

    public CoralAim(CoralDrive coralDrive) {
        this.coralDrive = coralDrive;
    }

    @Override
    public void execute() {
        double targetX = aprilLimelight.getTargetX();
        double targetV = aprilLimelight.getTargetVisible();

        System.out.println("Target X" + targetX);
        System.out.println("Target v" + targetV);
        // Return if april tag is not visible
        if (targetV == 0) {
            coralDrive.setOmega(0);
            coralDrive.setVelocityX(0);
            coralDrive.setVelocityY(0);
            System.out.println("Can not see note");
            return;
        }

        else 
            if (Math.abs(targetX) < tXRange) {
                System.out.println("----EQUAL------");
                coralDrive.setOmega(0);
                coralDrive.setVelocityX(0.3);
                coralDrive.setVelocityY(0);

            } else if (targetX < 0) { // if less than target
                coralDrive.setOmega(-rotationSpeed);
                System.out.println("target < 0");
            } else if (targetX > 0) { // if greater than target
                coralDrive.setOmega(rotationSpeed);
                System.out.println("target > 0");

            }

            laserTrigger.onTrue(Commands.runOnce(() -> {
                Index.get().stop();
                Intake.get().stop();
                coralDrive.setVelocityX(0);

            }));
        }

    public boolean isAligned() {
        double targetX = aprilLimelight.getTargetX();
        System.out.println("Target X = " + targetX);
        return Math.abs(targetX) < targetXRange;
    }
}