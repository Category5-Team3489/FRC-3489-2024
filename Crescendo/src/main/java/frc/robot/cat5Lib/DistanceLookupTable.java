package frc.robot.cat5lib;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class DistanceLookupTable {
    private static final double HistorySizeMeters = 1_000;

    private final TimeInterpolatableBuffer<Double> table = TimeInterpolatableBuffer
            .createDoubleBuffer(HistorySizeMeters);

    public DistanceLookupTable(Mapping... mappings) {
        for (Mapping mapping : mappings) {
            table.addSample(mapping.distanceMeters, mapping.outputValue);
        }
    }

    public double estimateOutputValue(double distanceMeters) {
        return table.getSample(distanceMeters).get();
    }

    public static final class Mapping {
        public final double distanceMeters;
        public final double outputValue;

        public Mapping(double distanceMeters, double outputValue) {
            this.distanceMeters = distanceMeters;
            this.outputValue = outputValue;
        }
    }

}
