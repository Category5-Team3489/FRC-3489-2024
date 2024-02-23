package frc.robot.enums;

public enum ShooterAngleState {
    //TODO Update values
    Start(0.0),
    Max(90.0);

    private final double shooterAngle;

    private ShooterAngleState(double shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    public double getAngle() {
        return shooterAngle;
    }

}