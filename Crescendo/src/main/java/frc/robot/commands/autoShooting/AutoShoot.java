package frc.robot.commands.autoShooting;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.enums.IndexState;
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
    

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(getDrivetrainVelocityX())
            .withVelocityY(getDrivetrainVelocityX())
            .withRotationalRate(getDrivetrainAngleRate()));

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double rotationSpeed = 0.4 * Constants.Drivetrain.MaxRadiansPerSecond;

    private final double targetXRange = 5;
    private final double maxYMeterRange = 4;
    private final double minYMeterRange = 0.2;

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
        // System.out.println("Lime Light pressed============================");

        
    }

    @Override
    public void execute() {
        // System.out.println("Lime light execute");
        double targetX = aprilLimelight.getTargetX();
        double targetY = aprilLimelight.getTargetY();
        

        

        // Return if april tag is not visible
        if (Double.isNaN(targetX) || Double.isNaN(targetY)) {
            drivetrainAngleRate = 0;
            drivetrainVelocityX = 0;
            drivetrainVelocityY = 0.5;
            return;
        }

        double angle = getShooterAngle(targetY);
        shooterAngle.updateCommand(angle);

        // System.out.println(angle + "ANGLE+++++++++++++++");

        // if the distance from april tag > Max values in table
        if (estimateFloorDistance(targetY) > maxYMeterRange) {
            drivetrainVelocityY = 0.2 * Constants.Drivetrain.MaxMetersPerSecond;      //drive forward
            System.out.println("if statment");

          //if robot is against Speaker
        } else if (estimateFloorDistance(targetY) < minYMeterRange) {
            //If x-value inside of range
            if (Math.abs(targetX) <= targetXRange) {
                // shoot note
                index.indexCommand(IndexState.Intake);
            }
            // if we are in the negative, move right
            else if (targetX < 0) {
                 drivetrainVelocityX = 0.2 * Constants.Drivetrain.MaxMetersPerSecond;
            } 
            // if we are in the positive, move left
            else if (targetX > 0) {
                 drivetrainVelocityX = -0.2 * Constants.Drivetrain.MaxMetersPerSecond;
            }
        }

        // set shooter angle based on ty calculation

        // April is visible
        // if (targetX > targetXRange) {
        // drivetrainAngleRate = -0.5;
        // } else if (targetX > targetXRange) {
        // drivetrainAngleRate = 0.5;
        // } else {
        // index.indexCommand(IndexState.Intake);
        // }

        // Apriltag is visible
        if (Math.abs(targetX) < targetXRange) {
            index.indexCommand(IndexState.Intake);
        } else if (targetX < 0) {
            drivetrainAngleRate = rotationSpeed;
        } else if (targetX > 0) {
            drivetrainAngleRate = rotationSpeed;
        }

    }

    private double getDrivetrainAngleRate() {
        return drivetrainAngleRate;
    }

    private double getDrivetrainVelocityX() {
        return drivetrainVelocityX;
    }

    private double getDrivetrainVelocityY() {
        return drivetrainVelocityY;
    }

    private double estimateFloorDistance(double targetY) {
        // TODO Fill in Daniel's fancy math
        return 8;
    }

    private double getShooterAngle(double ty) {
        // TODO fill in with shooter look up table
        return 35.8;
    }
}