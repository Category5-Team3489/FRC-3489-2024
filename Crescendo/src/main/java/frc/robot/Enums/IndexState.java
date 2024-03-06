package frc.robot.enums;

//TODO Update Speeds
public enum IndexState {
    Outtake(1),
    Intake(-1),
    Stop(0);

    private final double speedPercent;

    private IndexState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}