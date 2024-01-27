package frc.robot.Enums;

public enum BeltState {
    Belt_1(0.4),
    Belt_2(0.5),
    BeltShooter(0.6),
    BeltOut(-0.5),
    Belt_nomove(0);
    
    private final double beltEnumSpeed;

    private BeltState(double beltEnumSpeed) {
        this.beltEnumSpeed = beltEnumSpeed;
    }

    public double getbeltEnumSpeed() {
        return beltEnumSpeed;
    }

}

