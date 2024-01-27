package frc.robot.Enums;

public enum ShooterState {
    CloseSpeed(0.4),
    FarSpeed(0.6),
    IntakeSpeed(-0.5),
    Off(0);

    private final double shooterSpeed;

    private ShooterState(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }

    public double getSpeed() {
        return shooterSpeed;
    }

}
