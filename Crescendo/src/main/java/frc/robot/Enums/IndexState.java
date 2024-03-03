package frc.robot.enums;

//TODO Update Speeds
public enum IndexState {
    Outtake(0.6),
    Intake(-0.6),
    Stop(0);

    private final double speedPercent;

    private IndexState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}