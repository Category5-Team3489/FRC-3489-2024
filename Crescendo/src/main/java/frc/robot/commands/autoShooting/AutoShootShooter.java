package frc.robot.commands.autoShooting;

import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.DataPoint;
import frc.robot.ShooterSetting;

public class AutoShootShooter {
    //TODO Finish command

    public static ArrayList<DataPoint> getClosestDistanceIndicies(double distance) {
        ArrayList<DataPoint> indicies = new ArrayList<DataPoint>();
        for (DataPoint point : Constants.AprilLimelight.ShooterDistanceTable) {
            indicies.add(DataPoint.c(point.x, point.y - distance));
        }
        indicies.sort((a, b) -> DataPoint.compare(a, b));
        return indicies;
    }

    public static ShooterSetting getShooterSetting(double distance) {
        // get sorted list of distances
        ArrayList<DataPoint> indicies = getClosestDistanceIndicies(distance);
        // gets x values of the two closesed distances
        int closeDistance = indicies.get(0).x;
        int nextDistance = indicies.get(1).x;
        // gets y values (height) of the two closesed distances
        double nextHeight = Constants.AprilLimelight.ShooterDistanceTable[nextDistance].y;
        double closeHeight = Constants.AprilLimelight.ShooterDistanceTable[closeDistance].y;

        if (closeHeight < nextHeight) { // if the closesed distance's height < the second closesed distances height
            double d3 = nextHeight;
            nextHeight = closeHeight;
            closeHeight = d3;

            int c = closeDistance;
            closeDistance = nextDistance;
            nextDistance = c;
        }
        double t = (closeHeight - distance) / (closeHeight - nextHeight);

        ShooterSetting settingA = new ShooterSetting(
                Constants.AprilLimelight.ShooterSpeedAtDistanceTable[closeDistance],
                Constants.AprilLimelight.ShooterAngleAtDistanceTable[closeDistance]);
        ShooterSetting settingB = new ShooterSetting(
                Constants.AprilLimelight.ShooterSpeedAtDistanceTable[nextDistance],
                Constants.AprilLimelight.ShooterAngleAtDistanceTable[nextDistance]);

        return new ShooterSetting(Constants.General.lerp(settingA.shooterSpeed, settingB.shooterSpeed, t),
                Constants.General.lerp(settingA.shooterAngle, settingB.shooterAngle, t));
    }

}
