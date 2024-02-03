package frc.robot.enums;

public enum LimelightPipeline {
    Shooting(0),
    Camera(1);

    private final long index;

    private LimelightPipeline(long index) {
        this.index = index;
    }

    public long getIndex() {
        return index;
    }
}
