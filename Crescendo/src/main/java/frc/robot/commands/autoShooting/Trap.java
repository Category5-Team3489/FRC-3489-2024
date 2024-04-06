package frc.robot.commands.autoShooting;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooter.SetShooterSpeedAngleDifferent;
import frc.robot.enums.IndexState;
import frc.robot.subsystems.AprilLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;

public class Trap extends Command {
    private final AprilLimelight aprilLimelight = AprilLimelight.get();
    private final Drivetrain drivetrain = Drivetrain.get();
    private final ShooterAngle shooterAngle = ShooterAngle.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
    private final Index index = Index.get();

    Command setShooterTrap = new SetShooterSpeedAngleDifferent(
            Constants.ShooterAngle.TrapShooterAngle, Constants.ShooterSpeed.TrapTopShooterSpeed,
            Constants.ShooterSpeed.TrapTopShooterSpeed);

    final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    Command robotDriveCommand = drivetrain.applyRequest(() -> robotCentricDrive
            .withVelocityX(getDrivetrainVelocityX())
            .withVelocityY(getDrivetrainVelocityY())
            .withRotationalRate(getDrivetrainAngleRate()));

    Command indexCommand = index.indexCommand(IndexState.Intake);

    private Timer indexTimer = new Timer();
    private Timer shooterTimer = new Timer();

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;

    // TODO Test numbers
    private final double targetXRange = 10;
    private final double targetXRotationRange = 5;
    private final double targetYRange = 20;

    private final double maxYMeterRange = 4;
    private final double minYMeterRange = 0.2;

    public Trap() {

    }

    @Override
    public void initialize() {

        setShooterTrap.schedule();

        indexTimer.stop();
        shooterTimer.stop();

        indexTimer.reset();
        shooterTimer.reset();

        shooterTimer.start();
    }

    // press button
    // get limelight values
    // set shooter speed/angle
    // move to shooting position
    // index

    @Override
    public void execute() {
        double targetX = aprilLimelight.getTargetX();
        double targetY = aprilLimelight.getTargetY();
        double targetV = aprilLimelight.getTargetVisible();
        double targetS = aprilLimelight.getTargetS();

        // Return if april tag is not visible
        if (targetV == 0) {
            drivetrainAngleRate = 0;
            drivetrainVelocityX = 0;
            drivetrainVelocityY = 0;
            System.out.println("CAN NOT SEE APRIL TAG");
            return;
        }

        // Apriltag is visible
        if (Math.abs(targetX) < targetXRange) {
            drivetrainVelocityY = 0;
        } else if (targetX < 0) { // If robot is to left of tag:
            drivetrainVelocityY = 0.3;
            robotDriveCommand.schedule();
        } else if (targetX > 0) { // If robot is to right of tag:
            drivetrainVelocityY = -0.3;
            robotDriveCommand.schedule();
        }

        if (Math.abs(targetY) < targetYRange) {
            drivetrainVelocityX = 0;
        } else if (targetY < 0) { // If robot is to close to tag:
            drivetrainVelocityX = 0.3;
            robotDriveCommand.schedule();
        } else if (targetY > 0) { // If robot is to far from tag:
            drivetrainVelocityX = -0.3;
            robotDriveCommand.schedule();
        }

        if (Math.abs(targetX) < targetXRotationRange) {

            indexAfterShooterSpeed(0.8);
            robotDriveCommand.cancel();

        } else if (targetX < 0) {
            drivetrainAngleRate = -rotationSpeed;
            robotDriveCommand.schedule();
        } else if (targetX > 0) {
            drivetrainAngleRate = rotationSpeed;
            robotDriveCommand.schedule();
        }

        // if left of target
        // If to far
        //
        // If to close
        //
        // If ty within range
        //
        // if right of target
        // If to far
        // If to close
        // If within target

    }

    private void indexAfterShooterSpeed(double time) {
        if (shooterTimer.hasElapsed(time)) {
            indexTimer.start();
            if (!indexTimer.hasElapsed(2)) { // 2
                indexCommand.schedule();
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

    @Override
    public void end(boolean interrupted) {
        robotDriveCommand.cancel();
        indexCommand.cancel();
        System.out.println("INDEX Cancled");
        index.stop();
        System.out.println("Cancled");
    }

    @Override
    public boolean isFinished() {
        return indexTimer.hasElapsed(2);
    }

}