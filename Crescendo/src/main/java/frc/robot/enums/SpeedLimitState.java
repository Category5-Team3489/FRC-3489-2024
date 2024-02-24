package frc.robot.enums;
//TODO Update Speeds
public enum SpeedLimitState {
    Half(0.5),
    Full(1),
    Forth(0.25);

    private final double speedLimit;

    private SpeedLimitState(double speedLimit) {
        this.speedLimit = speedLimit;
    }

    public double getSpeedLimit() {
        return speedLimit;
    }

}