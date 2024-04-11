package frc.robot.commands.autoShooting;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
            .withVelocityY(getDrivetrainVelocityY())
            .withRotationalRate(getDrivetrainAngleRate()));

    Command indexCommand = index.indexCommand(IndexState.Intake);

    Command shootSpeed = shooterSpeed.updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent);

    Command angleCommand = shooterAngle
            .updateCommand(() -> getShooterAngle(estimateFloorDistance(aprilLimelight.getTargetY())));

    // Command waitIndexCommand = Commands.waitSeconds(4).andThen(indexCommand);

    private double angle = 35;

    private Timer indexTimer = new Timer();
    private Timer shooterTimer = new Timer();

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;

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
        // Commands.runOnce(() -> Commands.parallel(
        // shooterSpeed.updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent),
        // drivetrain.applyFieldCentricFacingAngle(() -> 0, () -> 0, () -> 0)));

    }

    @Override
    public void initialize() {
        double targetY = aprilLimelight.getTargetY();

        indexTimer.stop();
        shooterTimer.stop();

        indexTimer.reset();
        shooterTimer.reset();

        shooterTimer.start();

        driveCommandForward.schedule();

        shootSpeed.schedule();
        System.out.println("SHOOT SPEED START");
        // angle = getShooterAngle(targetY);
        angleCommand.schedule();
        System.out.println("Angle command start");
        // init drive command

    }

    @Override
    public void execute() {

        if (driveCommandForward.isScheduled()) {
            System.out.println("Scheduled");
        } else {
            System.out.println("Not Scheduled");
        }

        double targetX = aprilLimelight.getTargetX();
        double targetY = aprilLimelight.getTargetY();
        double targetV = aprilLimelight.getTargetVisible();

        double floorDistance = estimateFloorDistance(targetY);
        System.out.println("DISTANCE = " + floorDistance);

        // Return if april tag is not visible
        if (targetV == 0) {
            drivetrainAngleRate = 0;
            drivetrainVelocityX = -0.5;
            drivetrainVelocityY = 0;
            System.out.println("Double.isNaN(targetX) || Double.isNaN(targetY)");
            return;
        }

        // if the distance from april tag > Max values in table
        // TODO uncomment after testing
        // if (estimateFloorDistance(targetY) > maxYMeterRange) {
        // drivetrainVelocityY = 0.2 * Constants.Drivetrain.MaxMetersPerSecond; //drive
        // forward
        // System.out.println("if statment--estimateFloorDistance(targetY) >
        // maxYMeterRange");

        // //if robot is against Speaker
        // } else if (estimateFloorDistance(targetY) < minYMeterRange) {
        // //If x-value inside of range
        // if (Math.abs(targetX) <= targetXRange) {
        // // shoot note
        // index.indexCommand(IndexState.Intake);
        // }
        // // if we are in the negative, move right
        // else if (targetX < 0) {
        // drivetrainVelocityX = 0.2 * Constants.Drivetrain.MaxMetersPerSecond;
        // }
        // // if we are in the positive, move left
        // else if (targetX > 0) {
        // drivetrainVelocityX = -0.2 * Constants.Drivetrain.MaxMetersPerSecond;
        // }
        // }

        // set shooter angle based on ty calculation

        // Apriltag is visible
        // if the robot is rotated within 5 degrees of what it should be.
        if (Math.abs(targetX) < targetXRange) {

            indexAfterShooterSpeed(1.2); // Could adjust this value to decrease cycle time
            driveCommandForward.cancel();
            // if the ronot is too far left of april tag
        } else if (targetX < 0) {
            drivetrainAngleRate = -rotationSpeed;
            driveCommandForward.schedule();
            // if the robot is too far right of april tag
        } else if (targetX > 0) {
            drivetrainAngleRate = rotationSpeed;
            driveCommandForward.schedule();
        }

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

    public static double estimateFloorDistance(double targetY) {
        double distance = 44.06 / Math.tan(Math.toRadians(49 + targetY));
        System.out.println("Distance === " + distance);
        return distance;
    }

    public static double getShooterAngle(double distance) {
        // TODO fill in with shooter Math
        // return angle;
        return calculateTheta(83, distance);
    }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward.cancel();
        indexCommand.cancel();
        System.out.println("INDEX Cancled");
        shootSpeed.cancel();
        angleCommand.cancel();
        index.stop();
        System.out.println("Cancled");
    }

    @Override
    public boolean isFinished() {
        return indexTimer.hasElapsed(2);
    }

    // method that returns angle shooter needs to be at to shoot at target height,
    // pass in 81.5 as target height for our practice field
    public static double calculateTheta(double targetHeightAboveGround, double distanceBetweenTargetAndLimelight) {
        // declare values to be calculated
        double thetaRegularDegrees = 0;
        double theta = 0; // will end up with robot degrees here

        // constants used for math functions
        final double distanceBetweenLimelightAndPivot = 32.05; // distance between value returned by limelight and
                                                               // actual position of the pivot
        final double additionalPivotHorizontalOffset = -12.08; // this number is used to
        final double pivotHeight = 16.375; // constant distance in inches from ground to

        // calculate theta
        double oppositeSideOfTriangle = targetHeightAboveGround - pivotHeight;
        double adjacentSideOfTrinagle = distanceBetweenTargetAndLimelight + distanceBetweenLimelightAndPivot
                + additionalPivotHorizontalOffset;
        double thetaRadians = Math.atan(oppositeSideOfTriangle /
                adjacentSideOfTrinagle);
        thetaRegularDegrees = Math.toDegrees(thetaRadians);
        theta = ConvertRealDegreesToRobotDegrees(thetaRegularDegrees);

        // because limits are a good idea
        double maxRobotDegreeLimit = 90;
        double minRobotDegreeLimit = 5;

        return (MathUtil.clamp(theta, minRobotDegreeLimit, maxRobotDegreeLimit));
    }

    public static double ConvertRealDegreesToRobotDegrees(double realDegrees) {
        double offset = 11;
        double ratio = 1.238;
        double realDegreesMinusOffset = realDegrees - offset;

        double robotDegrees = realDegreesMinusOffset / ratio;
        return (robotDegrees);
    }

}