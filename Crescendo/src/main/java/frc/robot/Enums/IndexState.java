package frc.robot.enums;

public enum IndexState {
    Belt_1(0.4),
    Belt_2(0.5),
    BeltShooter(0.6),
    Outtake(-0.5),
    StopIndex(0);

    private final double indexEnumSpeed;

    private IndexState(double indexEnumSpeed) {
        this.indexEnumSpeed = indexEnumSpeed;
    }

    public double getIndexEnumSpeed() {
        return indexEnumSpeed;
    }

}
