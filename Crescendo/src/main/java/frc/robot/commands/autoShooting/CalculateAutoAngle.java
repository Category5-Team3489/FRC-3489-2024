package frc.robot.commands.autoShooting;

import edu.wpi.first.math.MathUtil;

public class CalculateAutoAngle {

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
