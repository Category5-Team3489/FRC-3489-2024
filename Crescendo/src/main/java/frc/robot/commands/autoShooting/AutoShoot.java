package frc.robot.commands.autoShooting;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class AutoShoot extends Command {
    private final AprilLimelight aprilLimelight = AprilLimelight.get();
    private final Drivetrain drivetrain = Drivetrain.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
    private final Index index = Index.get();

    private final Command driveCommand = drivetrain.applyFieldCentricFacingAngle(
            () -> getDrivetrainAngle(),
            () -> getDrivetrainVelocityX(),
            () -> getDrivetrainVelocityY());

    private double drivetrainAngle = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    // private final BangBangController

    public AutoShoot() {
        // In Parallel:
        // - Set shooter to constant speed
        // - Adjust robot heading towards value calculated from limelight
        // - Adjust shooter angle towards value calculated from lookup table
        // When Done:
        // - Index
        // - Delay Seconds
        // - Stop shooter
        // - Home shooter angle

        // TODO Continue
        Commands.runOnce(() -> Commands.parallel(
                shooterSpeed.updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent),
                drivetrain.applyFieldCentricFacingAngle(() -> 0, () -> 0, () -> 0)));
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double targetX = aprilLimelight.getTargetX();
        double targetY = aprilLimelight.getTargetY();

        // Return if april tag is not visible
        if (Double.isNaN(targetX) || Double.isNaN(targetY)) {
            drivetrainAngle = 0;
            drivetrainVelocityX = 0;
            drivetrainVelocityY = 0.5;
            return;
        }

        // April is visible

    }

    private double getDrivetrainAngle() {
        return drivetrainAngle;
    }

    private double getDrivetrainVelocityX() {
        return drivetrainVelocityX;
    }

    private double getDrivetrainVelocityY() {
        return drivetrainVelocityY;
    }

    private double estimateFloorDistance(double targetY) {
        // TODO Fill in Daniel's fancy math
        return 0;
    }
}