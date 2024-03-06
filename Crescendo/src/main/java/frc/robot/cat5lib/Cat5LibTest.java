package frc.robot.cat5Lib;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.cat5Lib.DistanceLookupTable.Mapping;

public class Cat5LibTest {
    public void run() {
        DataLogManager.logNetworkTables(false);
        DataLogManager.start();

        Mapping[] mappings = new Mapping[] {
                new Mapping(0, 80),
                new Mapping(1, 70),
                new Mapping(2, 60),
                new Mapping(3, 55),
                new Mapping(4, 50),
                new Mapping(5, 47.5),
                new Mapping(6, 45),
                new Mapping(7, 40),
                new Mapping(8, 30),
                new Mapping(9, 20),
                new Mapping(10, 10)
        };
        DistanceLookupTable lookupTable = new DistanceLookupTable(mappings);

        DoubleLogEntry logEntry = new DoubleLogEntry(DataLogManager.getLog(), "/testing/lookup-table");
        for (double distanceMeters = 0; distanceMeters < 10; distanceMeters += 0.01) {
            double estimatedOutputValue = lookupTable.estimateOutputValue(distanceMeters);
            // Time is normally calculated by the log entry, but here I am converting the
            // meters to seconds so the graph works correctly.
            long timestampMicroseconds = (long) (distanceMeters * 1_000_000.0);
            logEntry.append(estimatedOutputValue, timestampMicroseconds);
        }
    }
}
