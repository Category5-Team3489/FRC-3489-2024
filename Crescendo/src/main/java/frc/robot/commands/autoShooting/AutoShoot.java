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
            .withVelocityY(getDrivetrainVelocityY())
            .withRotationalRate(getDrivetrainAngleRate()));
            

    Command indexCommand = index.indexCommand(IndexState.Intake);

    Command shootSpeed = shooterSpeed.updateCommand(Constants.ShooterSpeed.DefaultSpeedPercent);

    Command angleCommand = shooterAngle
            .updateCommand(getShooterAngle(estimateFloorDistance(aprilLimelight.getTargetY())));

    Command waitIndexCommand = Commands.waitSeconds(4).andThen(indexCommand.withTimeout(3).andThen(() -> cancel()));

    private double angle = 35;

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

        driveCommandForward.schedule();

        shootSpeed.schedule();
        // angle = getShooterAngle(targetY);
        angleCommand.schedule();
        // init drive command

    }

    @Override
    public void execute() {

        if(driveCommandForward.isScheduled()) {
            System.out.println("Scheduled");
        } else {
            System.out.println("Not Scheduled");
        }

        double targetX = aprilLimelight.getTargetX();
        double targetY = aprilLimelight.getTargetY();
        double targetV = aprilLimelight.getTargetVisible();

        // Return if april tag is not visible
        if (targetV == 0) {
            drivetrainAngleRate = 0;
            drivetrainVelocityX = 0.5;
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
        if (Math.abs(targetX) < targetXRange) {
            // angleCommand.schedule();
            // indexCommand.schedule();
            waitIndexCommand.schedule();

            driveCommandForward.cancel();
            // System.out.println("index");

        } else if (targetX < 0) {
            drivetrainAngleRate = -rotationSpeed;
            driveCommandForward.schedule();
        } else if (targetX > 0) {
            drivetrainAngleRate = rotationSpeed;
            driveCommandForward.schedule();
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
        double distance = 67.625 / Math.tan(Math.toRadians(46.13 + targetY));
        System.out.println("Distance === " + distance);
        return distance;
    }

    private double getShooterAngle(double distance) {
        // TODO fill in with shooter Math
        // return angle;
        return 35;
    }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward.cancel();
        indexCommand.cancel();
        shootSpeed.cancel();
        angleCommand.cancel();
        System.out.println("Cancled");
    }

    // private void calculateThetaButton_Click()
    // {
    // // declare values to be calculated
    // double thetaPivotReferenceMethodRegularDegrees = 0;
    // double thetaPivotReferenceMethod = 0; // will end up with robot degrees here

    // // pull in values that will be used during calculation from form
    // double targetHeightAboveGround = double.Parse(targetHeightTextBox.Text); //
    // set to 81.5" for our practice field. may need to change for comp field / comp
    // practive field
    // double distanceBetweenLimelightAndPivot =
    // double.Parse(distanceBetweenLimelightAndPivotTextBox.Text); // horizontal
    // distance between value returned by limelight and actual position of the pivot
    // point on robot
    // double additionalPivotHorizontalOffset = -12.08; // this number is used to
    // account for offset associated with shooter wheels
    // double distanceBetweenLimelightAndTarget =
    // double.Parse(distanceBetweenTargetAndLimelightTextBox.Text); // get actual
    // distance from limelight for this variable
    // double pivotHeight = 16.375; // constant distance in inches from ground to
    // pivot point on our robot

    // // calculate theta
    // double oppositeSideOfTriangle = targetHeightAboveGround - pivotHeight;
    // double adjacentSideOfTrinagle = distanceBetweenLimelightAndTarget +
    // distanceBetweenLimelightAndPivot + additionalPivotHorizontalOffset;
    // double thetaPivotReferenceMethodRadians = Math.atan(oppositeSideOfTriangle /
    // adjacentSideOfTrinagle);
    // thetaPivotReferenceMethodRegularDegrees =
    // ConvertRadiansToDegrees(thetaPivotReferenceMethodRadians);
    // thetaPivotReferenceMethod =
    // ConvertRealDegreesToRobotDegrees(thetaPivotReferenceMethodRegularDegrees);

    // // because limits are a good idea
    // double maxRobotDegreeLimit = 90;
    // double minRobotDegreeLimit = 5;
    // if (thetaPivotReferenceMethod > maxRobotDegreeLimit) {
    // thetaPivotReferenceMethod = maxRobotDegreeLimit; }
    // else if (thetaPivotReferenceMethod < minRobotDegreeLimit) {
    // thetaPivotReferenceMethod = minRobotDegreeLimit; }

    // // update form
    // thetaPivotReferenceMethodTextBox.Text =
    // thetaPivotReferenceMethodRegularDegrees.ToString("n2");
    // thetaPivotReferenceMethodRobotDegreesTextBox.Text =
    // thetaPivotReferenceMethod.ToString("n2");
    // }

    // public double ConvertRadiansToDegrees(double radians)
    // {
    // double degrees = (180 / Math.PI) * radians;
    // return (degrees);
    // }

    // public double ConvertRealDegreesToRobotDegrees(double realDegrees)
    // {
    // double offset = 11;
    // double ratio = 1.238;
    // double realDegreesMinusOffset = realDegrees - offset;

    // double robotDegrees = realDegreesMinusOffset/ratio;
    // return (robotDegrees);
    // }

}