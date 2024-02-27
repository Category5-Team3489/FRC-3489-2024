// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DriverControllerPort = 0;
        public static final int ManipulatorControllerPort = 1;

    }

    public static class LedConstants {
        public static final int RightPort = 0;
        public static final int LeftPort = 1;
        public static final int Length = 162;

        public static final int CubeLEDButton = 7;
        public static final int ConeLEDButton = 8;
    }

    //TODO get values
    public static class ShooterSpeed {
        public static final double CloseShooterSpeed = 0.7;
        public static final double FarShooterSpeed = 0.7;

    }

    //TODO get values
    public static class ShooterAngle {
        public static final double CloseShooterAngle = 27.599;
        public static final double FarShooterAngle = 9.9;

    }

    public static class AprilLimelight {
        //TODO Update
        public static final double[] ShooterSpeedAtDistanceTable = { 3200, 3000, 3000, 3000, 3250, 3575, 3400, 3000, 3250, 3000, 3250, 3500 };
        public static final double[] ShooterAngleAtDistanceTable = { 70, 70, 50, 50, 50, 50, 60, 60,  60, 65, 65, 65 };
        public static final DataPoint[] ShooterDistanceTable = { DataPoint.c(0, 50), DataPoint.c(1, 60), DataPoint.c(2, 70.9), DataPoint.c(3, 77.4), DataPoint.c(4, 84.5), DataPoint.c(5, 92.4), DataPoint.c(6, 101.4), DataPoint.c(7, 108.5), DataPoint.c(8, 115.5), DataPoint.c(9, 123), DataPoint.c(10, 137), DataPoint.c(11, 154) };
    }

    public static class General {
        public static double lerp(double a, double b, double t) {
            return a + t * (b - a);
        }
    }
    
}