// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Nicholas was here
package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.cat5lib.DistanceLookupTable;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
    Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        DistanceLookupTable lookupTable = new DistanceLookupTable();

        DoubleLogEntry logEntry = new DoubleLogEntry(DataLogManager.getLog(), "/debug/lookup-table");
        for (double distanceMeters = 0; distanceMeters < 10; distanceMeters += 0.01) {
            double estimatedOutputValue = lookupTable.estimateOutputValue(distanceMeters);
            logEntry.append(estimatedOutputValue, (long) (distanceMeters * 1000000.0));
        }

        // edu.wpi.first.wpilibj.RobotBase.startRobot(Robot::new);
    }
}
