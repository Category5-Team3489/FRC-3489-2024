package frc.robot.enums;

public enum ShooterAngleState {
    //TODO Update values
    Start(5.0),
    Max(62.0);

    private final double shooterAngle;

    private ShooterAngleState(double shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    public double getAngle() {
        return shooterAngle;
    }

}