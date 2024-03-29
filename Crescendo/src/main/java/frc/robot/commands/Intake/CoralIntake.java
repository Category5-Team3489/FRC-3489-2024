package frc.robot.commands.Intake;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.CoralLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class CoralIntake extends Command {
    private final Drivetrain drivetrain = Drivetrain.get();
    private final CoralLimelight coralLimelight = CoralLimelight.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final Index index = Index.get();

    private final IntakeUntilDetection intake = new IntakeUntilDetection();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    Command driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(getDrivetrainVelocityX())
            .withVelocityY(getDrivetrainVelocityY())
            .withRotationalRate(getDrivetrainAngleRate()));

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double speed = (16.5 / 3.281) * 0.1;

    private double rotationSpeed = 0.4 * Constants.Drivetrain.MaxRadiansPerSecond;

    // TODO Test
    private final double tXRange = 5;
    private final double tYRange = 5;

    public CoralIntake() {
        Commands.runOnce(() -> Commands.parallel(
                drivetrain.applyFieldCentricFacingAngle(() -> 0, () -> 0, () -> 0)));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double targetX = coralLimelight.getTargetX();
        double targetY = coralLimelight.getTargetY();
        double targetV = coralLimelight.getTargetVisible();

        double currentDegrees = drivetrain.getCurrentHeading();

        // Return if april tag is not visible
        if (targetV == 0) {
            System.out.println("Note is not visible");
            return;
        }

        if (targetY <= tYRange) {
            // if within range
            if (Math.abs(targetX) < tXRange) {
                intake.schedule();
                drivetrainAngleRate = 0;
                drivetrainVelocityX = getXVelocityCalculation(currentDegrees);
                drivetrainVelocityY = getYVelocityCalculation(currentDegrees);
                driveCommandForward.schedule();
                System.out.println("EQUAL");

            } else if (targetX < 0) { // if less than target
                drivetrainAngleRate = -rotationSpeed;
                driveCommandForward.schedule();
                System.out.println("target < 0"); //only printed this
            } else if (targetX > 0) { // if greater than target
                drivetrainAngleRate = rotationSpeed;
                driveCommandForward.schedule();
                System.out.println("target > 0");

            }
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
        return 0;
    }

    private double getShooterAngle(double ty) {
        // TODO fill in with shooter look up table
        return 35.8;
    }

    private double getXVelocityCalculation(double angle) {
        return Math.sin(angle) * speed;
    }

    private double getYVelocityCalculation(double angle) {
        return Math.cos(angle) * speed;
    }
}

// press button
// check if note is visible
// if with in ty degree range
// rotate depending on tx value until tx == 0
// start intake
// drive forward
// wait for laser value then shooter = home
// if outside
// print error
// return

// is the current heading output correct?
//
