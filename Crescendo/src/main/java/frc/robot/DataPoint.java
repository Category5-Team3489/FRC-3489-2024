package frc.robot;

public class DataPoint {
    public final int x;
    public final double y;

    public DataPoint(int x, double y) {
        this.x = x;
        this.y = y;
    }

    public static DataPoint c(int x, double y) {
        return new DataPoint(x, y);
    }

    public static int compare(DataPoint a, DataPoint b) {
        if (Math.abs(a.y) > Math.abs(b.y))
            return 1;
        if (Math.abs(a.y) == Math.abs(b.y))
            return 0;
        return -1;
    }
}