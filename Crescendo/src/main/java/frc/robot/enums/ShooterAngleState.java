package frc.robot.enums;

public enum ShooterAngleState {
    // TODO Might be division
    Start(5.0 / 0.88216761184),
    Max(62.0 / 0.88216761184);

    private final double shooterAngle;

    private ShooterAngleState(double shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    public double getAngle() {
        return shooterAngle;
    }

}