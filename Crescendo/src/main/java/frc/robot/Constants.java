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
        public static final double CloseShooterSpeed = 0.6;
        public static final double FarShooterSpeed = 0.9;

    }

    //TODO get values
    public static class ShooterAngle {
        public static final double CloseShooterAngle = 60.0;
        public static final double FarShooterAngle = 45.0;

    }
}
