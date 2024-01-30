package frc.robot.Enums;

public enum ShooterState {
    CloseSpeed(0.4),
    FarSpeed(0.6),
    IntakeSpeed(-0.5),
    Off(0);

    private final double rotationsPerSecond;

    private ShooterState(double rotationsPerSecond) {
        this.rotationsPerSecond = rotationsPerSecond;
    }

    public double getRotationsPerSecond() {
        return rotationsPerSecond;
    }

}
